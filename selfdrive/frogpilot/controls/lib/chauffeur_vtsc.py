import math
import numpy as np

from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import DT_MDL

# If you also need to reuse "calculate_road_curvature" from frogpilot_utilities:
# from openpilot.selfdrive.frogpilot.frogpilot_utilities import calculate_road_curvature

# The minimal speed under which we won't do turn control.
# If v_ego < CRUISING_SPEED, we just reset or do no-op.
CRUISING_SPEED = 5.0  # [m/s] ~ 11.2 mph

def nonlinear_lat_accel(v_ego_ms: float, turn_aggressiveness: float = 1.0) -> float:
    """
    Returns a 'nonlinear' lateral acceleration limit based on vehicle speed in mph.
    The logistic curve peaks at ~2.8 m/s² for more comfortable cornering.
    """
    v_ego_mph = v_ego_ms * CV.MS_TO_MPH

    # Tweakable logistic function parameters:
    base = 1.5       # lower "floor" m/s²
    span = 2.2       # growth above base
    center = 35.0    # mph center point
    k = 0.10         # slope in the logistic function

    lat_acc = base + span / (1.0 + math.exp(-k * (v_ego_mph - center)))
    lat_acc = min(lat_acc, 2.8)   # clamp at 2.8 m/s²
    return lat_acc * turn_aggressiveness

def margin_time_fn(v_ego_ms: float) -> float:
    """
    A simple interpolation function to pick an 'early slow-down margin' that grows with speed.
    - ~1s at standstill to ~5s at highway speeds
    """
    # Speeds in m/s
    v_low = 0.0
    t_low = 1.0       # 1 s margin at standstill
    v_med = 15.0      # ~33.6 mph
    t_med = 3.0       # 3 s margin in mid-range
    v_high = 31.3     # ~70 mph
    t_high = 5.0      # 5 s margin at ~70 mph

    if v_ego_ms <= v_low:
        return t_low
    elif v_ego_ms >= v_high:
        return t_high
    elif v_ego_ms <= v_med:
        # Interpolate between t_low and t_med
        ratio = (v_ego_ms - v_low) / (v_med - v_low)
        return t_low + ratio * (t_med - t_low)
    else:
        # Interpolate between t_med and t_high
        ratio = (v_ego_ms - v_med) / (v_high - v_med)
        return t_med + ratio * (t_high - t_med)

def find_apexes(curvature: np.ndarray, threshold: float = 5e-5) -> list:
    """
    Identify local maxima in the curvature array. If curvature is
    above 'threshold' and forms a peak relative to neighbors, call it an apex.
    Lowered threshold so moderate curves actually register.
    """
    apex_indices = []
    for i in range(1, len(curvature) - 1):
        # Check if it's a local max
        if curvature[i] > threshold and curvature[i] >= curvature[i+1] and curvature[i] > curvature[i-1]:
            apex_indices.append(i)
    return apex_indices

class VisionTurnSpeedController:
    """
    Multi-step Vision Turn Speed Controller with:
      1) Nonlinear lat accel limit
      2) Apex detection and multi-step decel/spool logic
      3) Margin-based backward pass
      4) Standard backward pass
      5) Forward pass
    Final symmetrical jerk-limit to avoid harsh transitions.
    """

    def __init__(
        self,
        turn_smoothing_alpha=0.3,
        reaccel_alpha=0.2,
        low_lat_acc=0.20,    # below this, do minimal turn limiting
        high_lat_acc=0.40,   # above this, decelerate more
        max_decel=2.0,       # [m/s^2]
        max_jerk=2.0,        # [m/s^3]
    ):
        self.turn_smoothing_alpha = turn_smoothing_alpha
        self.reaccel_alpha = reaccel_alpha
        self.LOW_LAT_ACC = low_lat_acc
        self.HIGH_LAT_ACC = high_lat_acc
        self.MAX_DECEL = max_decel
        self.MAX_JERK = max_jerk

        self.current_decel = 0.0
        self.vtsc_target_prev = 0.0

        # We'll store multi-step speeds for the next ~10s (33 steps).
        self.planned_speeds = np.zeros(33, dtype=float)

        # Minimum steps needed for apex-based multi-step planning
        self.MIN_STEPS_FOR_APEX = 3

        # Keep track of curvature between calls
        self.previous_curvature = 0.0

    def reset(self, v_ego: float, curvature: float = 0.0) -> None:
        """
        Resets all internal states (e.g., after turning VTSC off or dropping below CRUISING_SPEED).
        """
        self.vtsc_target_prev = v_ego
        self.current_decel = 0.0
        self.previous_curvature = curvature
        self.planned_speeds[:] = v_ego

    def update(self, v_ego: float, modelData, turn_aggressiveness: float = 1.0) -> float:
        """
        Main update. Takes current speed and modelData, returns a jerk-limited speed target.
        :param v_ego: current speed [m/s]
        :param modelData: model predictions with orientationRate.z, velocity.x arrays
        :param turn_aggressiveness: scaling factor for cornering
        :return: speed limit [m/s] that your upstream code can feed to a long controller
        """
        dt = DT_MDL

        # If no valid model data or v_ego < CRUISING_SPEED, just do single-step fallback or reset
        orientationRate = getattr(modelData.orientationRate, "z", None)
        velocityPred = getattr(modelData.velocity, "x", None)

        # Guarantee we have arrays
        if orientationRate is None or velocityPred is None:
            # Hard fallback
            raw_target = self._single_step_fallback(v_ego, 0.0, turn_aggressiveness)
        else:
            orientationRate = np.array(orientationRate, dtype=float)
            velocityPred = np.array(velocityPred, dtype=float)

            # If they’re not the right length or v_ego < CRUISING_SPEED => fallback
            if len(orientationRate) < self.MIN_STEPS_FOR_APEX or len(velocityPred) < self.MIN_STEPS_FOR_APEX:
                raw_target = self._single_step_fallback(v_ego, 0.0, turn_aggressiveness)
            else:
                # *Multi-step plan*
                # 1) Upsample or slice to 33 if needed
                orientationRate_33 = self._maybe_upsample(orientationRate)
                velocityPred_33 = self._maybe_upsample(velocityPred)

                self.planned_speeds = self._plan_speed_trajectory(
                    orientationRate_33,
                    velocityPred_33,
                    init_speed=self.vtsc_target_prev,
                    turn_aggressiveness=turn_aggressiveness
                )
                raw_target = self.planned_speeds[0]

        # Final symmetrical jerk-limiting
        accel_cmd = (raw_target - self.vtsc_target_prev) / dt
        accel_cmd = clip(accel_cmd, -self.MAX_DECEL, self.MAX_DECEL)

        # Jerk-limit the change in acceleration
        accel_diff = accel_cmd - self.current_decel
        max_delta = self.MAX_JERK * dt
        if accel_diff > max_delta:
            self.current_decel += max_delta
        elif accel_diff < -max_delta:
            self.current_decel -= max_delta
        else:
            self.current_decel = accel_cmd

        # Final target after jerk limit
        jerk_limited_target = self.vtsc_target_prev + self.current_decel * dt

        # Update internal states
        self.vtsc_target_prev = jerk_limited_target

        return jerk_limited_target

    def _maybe_upsample(self, arr: np.ndarray, desired_len: int = 33) -> np.ndarray:
        """
        If arr has fewer than desired_len elements (e.g. 3..33), linearly interpolate up to desired_len.
        If arr has at least desired_len, slice the first desired_len elements.
        """
        n = len(arr)
        if n == desired_len:
            return arr
        elif n > desired_len:
            return arr[:desired_len]
        else:
            # Interpolate up to 33
            x_src = np.arange(n)
            x_dst = np.linspace(0, n - 1, desired_len)
            return np.interp(x_dst, x_src, arr)

    def _plan_speed_trajectory(
        self,
        orientation_rate: np.ndarray,
        velocity_pred: np.ndarray,
        init_speed: float,
        turn_aggressiveness: float
    ) -> np.ndarray:
        """
        Multi-step speed planning:
          1) Base safe speeds from lat accel
          2) Apex-based decel/spool
          3) Margin-based backward pass
          4) Standard backward pass
          5) Forward pass
        """
        n = len(orientation_rate)  # typically 33
        dt = DT_MDL

        # Compute curvature array:
        # curvature[i] = orientation_rate[i] / velocity_pred[i], with small epsilon safety
        eps = 1e-9
        curvature_array = np.array([
            orientation_rate[i] / max(velocity_pred[i], eps)
            for i in range(n)
        ], dtype=float)

        # 1) "Safe speeds" given lat accel limit
        safe_speeds = np.zeros(n, dtype=float)
        for i in range(n):
            lat_acc_limit = nonlinear_lat_accel(velocity_pred[i], turn_aggressiveness)
            c = max(curvature_array[i], 1e-9)
            safe_speeds[i] = math.sqrt(lat_acc_limit / c) if c > 1e-9 else 70.0
            safe_speeds[i] = clip(safe_speeds[i], 0.0, 70.0)

        # 2) Apex detection and decel/spool logic
        apex_idxs = find_apexes(curvature_array, threshold=5e-5)
        planned = safe_speeds.copy()

        for apex_i in apex_idxs:
            apex_speed = planned[apex_i]

            # Slightly larger decel/spool windows => decel sooner, spool earlier
            decel_factor = 0.15
            spool_factor = 0.08

            decel_window = int(max(2, (velocity_pred[apex_i] * decel_factor) / dt))
            spool_window = int(max(2, (velocity_pred[apex_i] * spool_factor) / dt))

            decel_start = max(0, apex_i - decel_window)
            spool_start = max(0, apex_i - spool_window)
            spool_end   = min(n, apex_i + spool_window + 1)

            # Ramp down to apex
            if spool_start > decel_start:
                v_decel_start = planned[decel_start]
                steps_decel = spool_start - decel_start
                for idx in range(decel_start, spool_start):
                    f = (idx - decel_start) / float(steps_decel)
                    planned[idx] = v_decel_start * (1 - f) + apex_speed * f

            # Keep speed at or below apex_speed up to apex_i
            for idx in range(spool_start, apex_i + 1):
                planned[idx] = min(planned[idx], apex_speed)

            # Spool-up after apex
            if spool_end > apex_i:
                steps_spool = spool_end - apex_i
                v_spool_end = planned[spool_end - 1]
                for idx in range(apex_i, spool_end):
                    f = (idx - apex_i) / float(steps_spool)
                    spool_val = apex_speed * (1 - f) + v_spool_end * f
                    planned[idx] = min(planned[idx], spool_val)

        # 3) Margin-based backward pass (decelerate earlier if needed)
        margin_t = margin_time_fn(init_speed)
        margin_steps = min(n - 1, int(margin_t / dt))
        accel_cmd = 0.0
        max_delta = self.MAX_JERK * dt
        for i in range(n - 1 - margin_steps, -1, -1):
            v_future = planned[i + margin_steps]
            time_future = margin_steps * dt
            err = planned[i] - v_future
            desired_acc = err / time_future

            accel_diff = desired_acc - accel_cmd
            if accel_diff > max_delta:
                accel_cmd += max_delta
            elif accel_diff < -max_delta:
                accel_cmd -= max_delta
            else:
                accel_cmd = desired_acc

            accel_cmd = clip(accel_cmd, -self.MAX_DECEL, self.MAX_DECEL)
            feasible_speed = v_future + accel_cmd * (-time_future)
            planned[i] = min(planned[i], feasible_speed, safe_speeds[i])

        # 4) Standard backward pass
        accel_cmd = 0.0
        for i in range(n - 2, -1, -1):
            v_next = planned[i + 1]
            err = planned[i] - v_next
            desired_acc = err / dt

            accel_diff = desired_acc - accel_cmd
            if accel_diff > max_delta:
                accel_cmd += max_delta
            elif accel_diff < -max_delta:
                accel_cmd -= max_delta
            else:
                accel_cmd = desired_acc

            accel_cmd = clip(accel_cmd, -self.MAX_DECEL, self.MAX_DECEL)
            feasible_speed = v_next + accel_cmd * (-dt)
            planned[i] = min(planned[i], feasible_speed, safe_speeds[i])

        # 5) Forward pass
        accel_cmd = 0.0
        err0 = planned[0] - init_speed
        desired_acc0 = err0 / dt
        accel_diff0 = desired_acc0 - accel_cmd
        if accel_diff0 > max_delta:
            accel_cmd += max_delta
        elif accel_diff0 < -max_delta:
            accel_cmd -= max_delta
        else:
            accel_cmd = desired_acc0
        accel_cmd = clip(accel_cmd, -self.MAX_DECEL, self.MAX_DECEL)
        planned[0] = init_speed + accel_cmd * dt
        planned[0] = min(planned[0], safe_speeds[0])

        for i in range(1, n):
            v_prev = planned[i - 1]
            err = planned[i] - v_prev
            desired_acc = err / dt

            accel_diff = desired_acc - accel_cmd
            if accel_diff > max_delta:
                accel_cmd += max_delta
            elif accel_diff < -max_delta:
                accel_cmd -= max_delta
            else:
                accel_cmd = desired_acc

            accel_cmd = clip(accel_cmd, -self.MAX_DECEL, self.MAX_DECEL)
            planned[i] = v_prev + accel_cmd * dt
            planned[i] = min(planned[i], safe_speeds[i])

        return planned

    def _single_step_fallback(self, v_ego, curvature, turn_aggressiveness):
        """
        If we can't run multi-step planning, do a simpler approach: 
        just clamp speed to sqrt(lat_acc / curvature).
        Smooth or re-accel based on current lat acc vs. thresholds.
        """
        lat_acc = nonlinear_lat_accel(v_ego, turn_aggressiveness)
        c = max(curvature, 1e-9)
        v_curvature_ms = math.sqrt(lat_acc / c) if c > 1e-9 else 70.0
        v_curvature_ms = clip(v_curvature_ms, 0.0, 70.0)

        # Actual lateral accel with current speed:
        current_lat_acc = curvature * (v_ego ** 2)

        # If we are in a moderately high lateral accel region, blend down
        if current_lat_acc > self.LOW_LAT_ACC:
            if current_lat_acc < self.HIGH_LAT_ACC:
                alpha = 0.1  # gentle
            else:
                alpha = self.turn_smoothing_alpha  # stronger decel
            raw_target = alpha * self.vtsc_target_prev + (1.0 - alpha) * v_curvature_ms
        else:
            # re-accel if we are under lat limit
            alpha = self.reaccel_alpha
            raw_target = alpha * self.vtsc_target_prev + (1.0 - alpha) * v_curvature_ms

        # small apex nudge if curvature is dropping from previous
        if curvature < self.previous_curvature:
            apex_alpha = 0.2
            raw_target = (1.0 - apex_alpha) * raw_target + apex_alpha * v_curvature_ms

        self.previous_curvature = curvature
        return raw_target