import math
import numpy as np
import cereal.messaging as messaging

from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.selfdrive.modeld.constants import ModelConstants

CRUISING_SPEED = 5.0  # m/s

def nonlinear_lat_accel(v_ego_ms: float, turn_aggressiveness: float = 1.0) -> float:
    v_ego_mph = v_ego_ms * CV.MS_TO_MPH
    base = 1.5
    span = 2.2
    center = 35.0
    k = 0.10
    lat_acc = base + span / (1.0 + math.exp(-k * (v_ego_mph - center)))
    lat_acc = min(lat_acc, 2.8)
    return lat_acc * turn_aggressiveness

def margin_time_fn(v_ego_ms: float) -> float:
    v_low = 0.0
    t_low = 1.0
    v_med = 15.0     # ~34 mph
    t_med = 3.0
    v_high = 31.3    # ~70 mph
    t_high = 5.0

    if v_ego_ms <= v_low:
        return t_low
    elif v_ego_ms >= v_high:
        return t_high
    elif v_ego_ms <= v_med:
        ratio = (v_ego_ms - v_low) / (v_med - v_low)
        return t_low + ratio * (t_med - t_low)
    else:
        ratio = (v_ego_ms - v_med) / (v_high - v_med)
        return t_med + ratio * (t_high - t_med)

def find_apexes(curv_array: np.ndarray, threshold: float = 5e-5) -> list:
    apex_indices = []
    for i in range(1, len(curv_array) - 1):
        if (
            curv_array[i] > threshold
            and curv_array[i] >= curv_array[i + 1]
            and curv_array[i] > curv_array[i - 1]
        ):
            apex_indices.append(i)
    return apex_indices

# --------------------
# DYNAMIC SCALING LOGIC
# --------------------
def dynamic_decel_scale(v_ego_ms: float) -> float:
    """
    Example: at lower speeds, we want up to 4x 'faster' deceleration (and acceleration),
    but at highway speeds, 1x is enough. Adjust these to taste.
    """
    min_speed = 5.0   # below this speed => max scaling
    max_speed = 25.0  # above this speed => no extra scaling
    if v_ego_ms <= min_speed:
        return 4.0
    elif v_ego_ms >= max_speed:
        return 1.0
    else:
        # linear interpolation from 4.0 -> 1.0
        ratio = (v_ego_ms - min_speed) / (max_speed - min_speed)
        return 4.0 + (1.0 - 4.0) * ratio

def dynamic_jerk_scale(v_ego_ms: float) -> float:
    """
    Could be the same or a different function than decel. For clarity,
    we'll keep it the same. Tweak as needed.
    """
    return dynamic_decel_scale(v_ego_ms)
# --------------------

class VisionTurnSpeedController:
    def __init__(
        self,
        turn_smoothing_alpha=0.3,
        reaccel_alpha=0.2,
        low_lat_acc=0.20,
        high_lat_acc=0.40,
        max_decel=2.5,
        max_jerk=5.0
    ):
        self.turn_smoothing_alpha = turn_smoothing_alpha
        self.reaccel_alpha = reaccel_alpha
        self.LOW_LAT_ACC = low_lat_acc
        self.HIGH_LAT_ACC = high_lat_acc
        # Base decel/jerk
        self.MAX_DECEL = max_decel
        self.MAX_JERK = max_jerk

        self.current_accel = 0.0
        self.prev_target_speed = 0.0

        self.planned_speeds = np.zeros(33, dtype=float)

        self.sm = messaging.SubMaster(['modelV2'])

        self.prev_v_cruise_cluster = 0.0

    def reset(self, v_ego: float) -> None:
        self.prev_target_speed = v_ego
        self.current_accel = 0.0
        self.planned_speeds[:] = v_ego

    def update(self, v_ego: float, v_cruise_cluster: float, turn_aggressiveness=1.0) -> float:
        self.sm.update()
        modelData = self.sm['modelV2']

        if v_ego < CRUISING_SPEED:
            self.reset(v_ego)
            self.prev_v_cruise_cluster = v_cruise_cluster
            return v_ego

        orientation_rate_raw = modelData.orientationRate.z
        velocity_pred_raw = modelData.velocity.x

        MIN_POINTS = 3
        if (
            orientation_rate_raw is None or velocity_pred_raw is None
            or len(orientation_rate_raw) < MIN_POINTS
            or len(velocity_pred_raw) < MIN_POINTS
        ):
            raw_target = self._single_step_fallback(v_ego, 0.0, turn_aggressiveness)
        else:
            orientation_rate = np.array(orientation_rate_raw, dtype=float)
            orientation_rate = np.abs(orientation_rate)
            velocity_pred = np.array(velocity_pred_raw, dtype=float)

            n_points = min(len(orientation_rate), len(velocity_pred))
            if n_points < 33:
                src_indices = np.linspace(0, n_points - 1, n_points)
                dst_indices = np.linspace(0, n_points - 1, 33)
                orientation_rate_33 = np.interp(dst_indices, src_indices, orientation_rate[:n_points])
                velocity_pred_33 = np.interp(dst_indices, src_indices, velocity_pred[:n_points])
            else:
                orientation_rate_33 = orientation_rate[:33]
                velocity_pred_33 = velocity_pred[:33]

            times_33 = np.array(ModelConstants.T_IDXS[:33], dtype=float)

            self.planned_speeds = self._plan_speed_trajectory(
                orientation_rate_33,
                velocity_pred_33,
                times_33,
                init_speed=self.prev_target_speed,
                turn_aggressiveness=turn_aggressiveness
            )
            raw_target = self.planned_speeds[0]

        # ---- NEW: Always clamp raw_target to v_cruise_cluster so we don't do "relative" logic
        raw_target = min(raw_target, v_cruise_cluster)

        dt = 0.05  # ~20Hz
        final_target_speed = None

        # If the driver just bumped up the dash speed:
        if v_cruise_cluster > self.prev_v_cruise_cluster:
            # Let them accelerate immediately, up to the lesser of:
            # new cluster speed, or safe curve speed
            final_target_speed = min(v_cruise_cluster, raw_target)
            self.current_accel = 0.0  # optional "fresh start"

        # If no cluster bump up, do symmetrical jerk-limit with dynamic scaling:
        if final_target_speed is None:
            accel_cmd = (raw_target - self.prev_target_speed) / dt

            # -- DYNAMIC SCALING --
            scale_decel = dynamic_decel_scale(v_ego)
            scale_jerk = dynamic_jerk_scale(v_ego)

            # clip acceleration command using scaled decel
            accel_cmd = clip(accel_cmd,
                             -self.MAX_DECEL * scale_decel,
                             self.MAX_DECEL * scale_decel)

            # jerk-limit
            accel_diff = accel_cmd - self.current_accel
            max_delta = (self.MAX_JERK * scale_jerk) * dt

            if accel_diff > max_delta:
                self.current_accel += max_delta
            elif accel_diff < -max_delta:
                self.current_accel -= max_delta
            else:
                self.current_accel = accel_cmd

            final_target_speed = self.prev_target_speed + self.current_accel * dt

        # Always clamp final to the current dash set speed
        final_target_speed = min(final_target_speed, v_cruise_cluster)

        self.prev_target_speed = final_target_speed
        self.prev_v_cruise_cluster = v_cruise_cluster
        return final_target_speed

    def _plan_speed_trajectory(
        self,
        orientation_rate: np.ndarray,
        velocity_pred: np.ndarray,
        times: np.ndarray,
        init_speed: float,
        turn_aggressiveness: float
    ) -> np.ndarray:
        n = len(orientation_rate)
        eps = 1e-9
        dt_array = np.diff(times)

        curvature = np.array([
            orientation_rate[i] / max(velocity_pred[i], eps)
            for i in range(n)
        ], dtype=float)

        safe_speeds = np.zeros(n, dtype=float)
        for i in range(n):
            lat_acc_limit = nonlinear_lat_accel(velocity_pred[i], turn_aggressiveness)
            c = max(curvature[i], 1e-9)
            safe_speeds[i] = math.sqrt(lat_acc_limit / c) if c > 1e-9 else 70.0
            safe_speeds[i] = clip(safe_speeds[i], 0.0, 70.0)

        apex_idxs = find_apexes(curvature, threshold=5e-5)
        planned = safe_speeds.copy()
        for apex_i in apex_idxs:
            apex_speed = planned[apex_i]

            decel_factor = 0.15
            spool_factor = 0.08

            decel_sec = velocity_pred[apex_i] * decel_factor
            spool_sec = velocity_pred[apex_i] * spool_factor

            decel_start = self._find_time_index(times, times[apex_i] - decel_sec)
            spool_start = self._find_time_index(times, times[apex_i] - spool_sec)
            spool_end   = self._find_time_index(times, times[apex_i] + spool_sec, clip_high=True)

            if spool_start > decel_start:
                v_decel_start = planned[decel_start]
                steps_decel = spool_start - decel_start
                for idx in range(decel_start, spool_start):
                    f = (idx - decel_start) / float(steps_decel)
                    planned[idx] = v_decel_start * (1 - f) + apex_speed * f

            for idx in range(spool_start, apex_i + 1):
                planned[idx] = min(planned[idx], apex_speed)

            if spool_end > apex_i:
                steps_spool = spool_end - apex_i
                v_spool_end = planned[spool_end - 1]
                for idx in range(apex_i, spool_end):
                    f = (idx - apex_i) / float(steps_spool)
                    spool_val = apex_speed * (1 - f) + v_spool_end * f
                    planned[idx] = min(planned[idx], spool_val)

        margin_t = margin_time_fn(init_speed)
        new_planned = planned.copy()

        # Margin-based backward pass
        for i in range(n - 2, -1, -1):
            j = self._find_time_index(times, times[i] + margin_t, clip_high=True)
            if j <= i:
                continue
            dt_ij = times[j] - times[i]
            if dt_ij < 0.001:
                continue
            v_future = new_planned[j]
            err = new_planned[i] - v_future
            desired_acc = clip(err / dt_ij, -self.MAX_DECEL, self.MAX_DECEL)
            feasible_speed = v_future - desired_acc * dt_ij
            new_planned[i] = min(new_planned[i], feasible_speed, planned[i])

        # Standard backward pass
        dt_len = len(dt_array)
        for i in range(n - 2, -1, -1):
            if i < dt_len:
                dt_i = dt_array[i]
            else:
                dt_i = 0.05
            v_next = new_planned[i + 1]
            err = new_planned[i] - v_next
            desired_acc = clip(err / dt_i, -self.MAX_DECEL, self.MAX_DECEL)
            feasible_speed = v_next - desired_acc * dt_i
            new_planned[i] = min(new_planned[i], feasible_speed, planned[i])

        # Forward pass
        new_planned[0] = min(new_planned[0], planned[0])
        dt_0 = dt_array[0] if dt_len > 0 else 0.05
        err0 = new_planned[0] - init_speed
        accel0 = clip(err0 / dt_0, -self.MAX_DECEL, self.MAX_DECEL)
        new_planned[0] = init_speed + accel0 * dt_0

        for i in range(1, n):
            dt_i = dt_array[i - 1] if i - 1 < dt_len else 0.05
            v_prev = new_planned[i - 1]
            err = new_planned[i] - v_prev
            desired_acc = clip(err / dt_i, -self.MAX_DECEL, self.MAX_DECEL)
            feasible_speed = v_prev + desired_acc * dt_i
            new_planned[i] = min(new_planned[i], feasible_speed, planned[i])

        return new_planned

    def _find_time_index(self, times: np.ndarray, target_time: float, clip_high=False) -> int:
        n = len(times)
        if target_time <= times[0]:
            return 0
        if target_time >= times[-1] and clip_high:
            return n - 1
        for i in range(n - 1):
            if times[i] <= target_time < times[i + 1]:
                if (target_time - times[i]) < (times[i + 1] - target_time):
                    return i
                else:
                    return i + 1
        return n - 1 if clip_high else n - 2

    def _single_step_fallback(self, v_ego, curvature, turn_aggressiveness):
        lat_acc = nonlinear_lat_accel(v_ego, turn_aggressiveness)
        c = max(curvature, 1e-9)
        safe_speed = math.sqrt(lat_acc / c) if c > 1e-9 else 70.0
        safe_speed = clip(safe_speed, 0.0, 70.0)

        current_lat_acc = curvature * (v_ego ** 2)
        if current_lat_acc > self.LOW_LAT_ACC:
            if current_lat_acc < self.HIGH_LAT_ACC:
                alpha = 0.1
            else:
                alpha = self.turn_smoothing_alpha
            raw_target = alpha * self.prev_target_speed + (1 - alpha) * safe_speed
        else:
            alpha = self.reaccel_alpha
            raw_target = alpha * self.prev_target_speed + (1 - alpha) * safe_speed

        return raw_target
