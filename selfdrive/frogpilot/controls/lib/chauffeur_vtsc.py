import math
import numpy as np
import cereal.messaging as messaging

from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.selfdrive.modeld.constants import ModelConstants

# A minimal speed, below which we do no special turn control
CRUISING_SPEED = 5.0  # m/s

def nonlinear_lat_accel(v_ego_ms: float, turn_aggressiveness: float = 1.0) -> float:
    """
    Returns a 'nonlinear' lateral acceleration limit based on vehicle speed in mph.
    The logistic curve peaks around ~2.8 m/s² for comfortable cornering.
    """
    v_ego_mph = v_ego_ms * CV.MS_TO_MPH

    base = 1.5
    span = 2.2
    center = 35.0
    k = 0.10

    lat_acc = base + span / (1.0 + math.exp(-k * (v_ego_mph - center)))
    lat_acc = min(lat_acc, 2.8)
    return lat_acc * turn_aggressiveness

def margin_time_fn(v_ego_ms: float) -> float:
    """
    A simple function to pick an 'early slow-down margin' that grows with speed.
    E.g. 1–3 seconds at low speeds up to 5 seconds at highway speeds.
    """
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
    """
    Identify local maxima in the curvature array above a threshold.
    """
    apex_indices = []
    for i in range(1, len(curv_array) - 1):
        if (
            curv_array[i] > threshold
            and curv_array[i] >= curv_array[i + 1]
            and curv_array[i] > curv_array[i - 1]
        ):
            apex_indices.append(i)
    return apex_indices

class VisionTurnSpeedController:
    """
    VTSC that respects the non-uniform times from T_IDXS. Instead of
    assuming a constant dt, we fetch times[i+1] - times[i].
    """

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
        self.MAX_DECEL = max_decel
        self.MAX_JERK = max_jerk

        self.current_accel = 0.0
        self.prev_target_speed = 0.0

        # We'll store the planned speeds for up to 33 steps
        self.planned_speeds = np.zeros(33, dtype=float)

        # Add subscriber for model data
        self.sm = messaging.SubMaster(['modelV2'])

        # 1) Add a field to store the last known v_cruise
        self.prev_v_cruise = 0.0

    def reset(self, v_ego: float) -> None:
        self.prev_target_speed = v_ego
        self.current_accel = 0.0
        self.planned_speeds[:] = v_ego

    # 2) Pass v_cruise into update(...)
    def update(self, v_ego: float, v_cruise: float, turn_aggressiveness=1.0) -> float:
        """
        Main update. We expect:
          modelData.orientationRate.z : array of length 33
          modelData.velocity.x        : array of length 33
          modelData.times             : array of length 33
        """
        self.sm.update()
        modelData = self.sm['modelV2']

        # If speed is below CRUISING_SPEED, do no special turn limit
        if v_ego < CRUISING_SPEED:
            self.reset(v_ego)
            self.prev_v_cruise = v_cruise  # Track the user's current cruise
            return v_ego

        # Convert the orientationRate and velocity arrays from Cap'n Proto to NumPy:
        orientation_rate_raw = modelData.orientationRate.z
        velocity_pred_raw = modelData.velocity.x

        # If either is None or too short, fall back to simpler logic
        MIN_POINTS = 3
        if (
            orientation_rate_raw is None or velocity_pred_raw is None
            or len(orientation_rate_raw) < MIN_POINTS
            or len(velocity_pred_raw) < MIN_POINTS
        ):
            raw_target = self._single_step_fallback(v_ego, 0.0, turn_aggressiveness)
        else:
            # Make sure these are NumPy arrays (float)
            orientation_rate = np.array(orientation_rate_raw, dtype=float)
            orientation_rate = np.abs(orientation_rate)
            velocity_pred = np.array(velocity_pred_raw, dtype=float)

            n_points = min(len(orientation_rate), len(velocity_pred))
            if n_points < 33:
                # Interpolate up to 33 points if needed
                src_indices = np.linspace(0, n_points - 1, n_points)
                dst_indices = np.linspace(0, n_points - 1, 33)
                orientation_rate_33 = np.interp(dst_indices, src_indices, orientation_rate[:n_points])
                velocity_pred_33 = np.interp(dst_indices, src_indices, velocity_pred[:n_points])
            else:
                orientation_rate_33 = orientation_rate[:33]
                velocity_pred_33 = velocity_pred[:33]

            # Convert times to float array, use first 33
            times_33 = np.array(ModelConstants.T_IDXS[:33], dtype=float)

            # Plan the speed trajectory
            self.planned_speeds = self._plan_speed_trajectory(
                orientation_rate_33,
                velocity_pred_33,
                times_33,
                init_speed=self.prev_target_speed,
                turn_aggressiveness=turn_aggressiveness
            )
            raw_target = self.planned_speeds[0]

        dt = 0.05  # ~20Hz
        final_target_speed = None

        # 3) USER-INITIATED v_cruise INCREASE CHECK
        if v_cruise > self.prev_v_cruise:
            # The user is "bumping up" the set speed → skip jerk limit and
            # immediately allow up to min(v_cruise, raw_target).
            final_target_speed = min(v_cruise, raw_target)
            # Optionally reset self.current_accel so next tick starts fresh:
            self.current_accel = 0.0

        # 4) If not an increase, proceed with normal jerk-limited approach
        if final_target_speed is None:
            accel_cmd = (raw_target - self.prev_target_speed) / dt
            accel_cmd = clip(accel_cmd, -self.MAX_DECEL, self.MAX_DECEL)

            # Jerk-limit the change in acceleration
            accel_diff = accel_cmd - self.current_accel
            max_delta = self.MAX_JERK * dt
            if accel_diff > max_delta:
                self.current_accel += max_delta
            elif accel_diff < -max_delta:
                self.current_accel -= max_delta
            else:
                self.current_accel = accel_cmd

            final_target_speed = self.prev_target_speed + self.current_accel * dt

        self.prev_target_speed = final_target_speed
        self.prev_v_cruise = v_cruise

        return final_target_speed

    def _plan_speed_trajectory(
        self,
        orientation_rate: np.ndarray,
        velocity_pred: np.ndarray,
        times: np.ndarray,
        init_speed: float,
        turn_aggressiveness: float
    ) -> np.ndarray:
        """
        1) Compute curvature for each time step
        2) Base "safe speed" from lat accel
        3) Apex-based decel/spool
        4) Margin-based backward pass (time-based)
        5) Standard backward pass (time-based)
        6) Forward pass (time-based)
        """
        n = len(orientation_rate)
        eps = 1e-9

        # times[i+1] - times[i] may vary!
        dt_array = np.diff(times)  # length n-1

        # 1) Curvature
        curvature = np.array([
            orientation_rate[i] / max(velocity_pred[i], eps)
            for i in range(n)
        ], dtype=float)

        # 2) Compute a safe speed for each step, based on lateral acceleration
        safe_speeds = np.zeros(n, dtype=float)
        for i in range(n):
            lat_acc_limit = nonlinear_lat_accel(velocity_pred[i], turn_aggressiveness)
            c = max(curvature[i], 1e-9)
            safe_speeds[i] = math.sqrt(lat_acc_limit / c) if c > 1e-9 else 70.0
            safe_speeds[i] = clip(safe_speeds[i], 0.0, 70.0)

        # 3) Apex detection + decel/spool handling
        apex_idxs = find_apexes(curvature, threshold=5e-5)
        planned = safe_speeds.copy()

        for apex_i in apex_idxs:
            apex_speed = planned[apex_i]

            # Factors controlling how far before/after apex we decel/spool
            decel_factor = 0.15
            spool_factor = 0.08

            # Convert speed-based factor to approximate # of seconds
            decel_sec = velocity_pred[apex_i] * decel_factor
            spool_sec = velocity_pred[apex_i] * spool_factor

            decel_start = self._find_time_index(times, times[apex_i] - decel_sec)
            spool_start = self._find_time_index(times, times[apex_i] - spool_sec)
            spool_end   = self._find_time_index(times, times[apex_i] + spool_sec, clip_high=True)

            # Ramp down to apex from decel_start..spool_start
            if spool_start > decel_start:
                v_decel_start = planned[decel_start]
                steps_decel = spool_start - decel_start
                for idx in range(decel_start, spool_start):
                    f = (idx - decel_start) / float(steps_decel)
                    planned[idx] = v_decel_start * (1 - f) + apex_speed * f

            # Force apex speed from spool_start..apex_i
            for idx in range(spool_start, apex_i + 1):
                planned[idx] = min(planned[idx], apex_speed)

            # Spool up after apex
            if spool_end > apex_i:
                steps_spool = spool_end - apex_i
                v_spool_end = planned[spool_end - 1]
                for idx in range(apex_i, spool_end):
                    f = (idx - apex_i) / float(steps_spool)
                    spool_val = apex_speed * (1 - f) + v_spool_end * f
                    planned[idx] = min(planned[idx], spool_val)

        # 4) Margin-based backward pass
        margin_t = margin_time_fn(init_speed)
        new_planned = planned.copy()

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

        # 5) Standard backward pass (time-based)
        dt_len = len(dt_array)
        for i in range(n - 2, -1, -1):
            if i < dt_len:
                dt_i = dt_array[i]
            else:
                dt_i = 0.05  # fallback
            v_next = new_planned[i + 1]
            err = new_planned[i] - v_next
            desired_acc = clip(err / dt_i, -self.MAX_DECEL, self.MAX_DECEL)
            feasible_speed = v_next - desired_acc * dt_i
            new_planned[i] = min(new_planned[i], feasible_speed, planned[i])

        # 6) Forward pass (time-based)
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
        """
        Returns the index i in 'times' such that times[i] is closest to target_time,
        without exceeding it (unless clip_high=True).
        """
        n = len(times)
        if target_time <= times[0]:
            return 0
        if target_time >= times[-1] and clip_high:
            return n - 1

        for i in range(n - 1):
            if times[i] <= target_time < times[i + 1]:
                # pick whichever boundary is closer
                if (target_time - times[i]) < (times[i + 1] - target_time):
                    return i
                else:
                    return i + 1
        return n - 1 if clip_high else n - 2

    def _single_step_fallback(self, v_ego, curvature, turn_aggressiveness):
        """
        Fallback if we only have a single 'curvature' float instead of full arrays.
        """
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
