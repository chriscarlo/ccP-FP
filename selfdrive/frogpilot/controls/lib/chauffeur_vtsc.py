import math
import numpy as np

from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import DT_MDL

def nonlinear_lat_accel(v_ego_ms: float, turn_aggressiveness: float = 1.0) -> float:

    """
    Returns a 'nonlinear' lateral acceleration limit based on vehicle speed in mph.
    We slightly adjusted constants to provide a more robust final clamp and
    a gentler logistic curve, mimicking an F1 driver’s willingness to corner
    firmly, but with a real-world safety margin.
    """

    v_ego_mph = v_ego_ms * CV.MS_TO_MPH
    base = 1.5
    span = 2.1
    center = 40.0
    k = 0.12
    lat_acc = base + span / (1.0 + math.exp(-k * (v_ego_mph - center)))
    lat_acc = min(lat_acc, 3.4)
    return lat_acc * turn_aggressiveness

def find_apexes(curvature: np.ndarray, threshold: float = 1e-4) -> list:

    """
    Identify indices of "apexes" (local maxima in curvature).
    An apex is a point where curvature[i] is a local maximum
    above 'threshold'. We also check that curvature is truly
    peaking (i.e., difference on either side) to reduce noise.
    """

    apex_indices = []
    for i in range(1, len(curvature) - 1):
        if (curvature[i] > curvature[i - 1]) and (curvature[i] >= curvature[i + 1]) \
           and (curvature[i] > threshold):
            if ((curvature[i] - curvature[i - 1]) > 0.5 * threshold) and \
               ((curvature[i] - curvature[i + 1]) > 0.5 * threshold):
                apex_indices.append(i)
    return apex_indices

class VisionTurnSpeedController:
    """
    A multi-step Vision Turn Speed Controller (VTSC) that can now handle
    any number of future timesteps m (≥3). If m < 3, we fallback. If 3 <= m < 33,
    we upsample to 33. If m >= 33, we just slice the first 33 points.

    All external method signatures remain identical, so this is a drop-in
    replacement.
    """

    def __init__(
        self,
        turn_smoothing_alpha=0.3,
        reaccel_alpha=0.2,
        low_lat_acc=0.20,
        high_lat_acc=0.40,
        max_decel=2.0,  # [m/s^2]
        max_jerk=2.0,   # [m/s^3]
    ):
        self.turn_smoothing_alpha = turn_smoothing_alpha
        self.reaccel_alpha = reaccel_alpha
        self.LOW_LAT_ACC = low_lat_acc
        self.HIGH_LAT_ACC = high_lat_acc
        self.MAX_DECEL = max_decel
        self.MAX_JERK = max_jerk

        self.current_decel = 0.0
        self.vtsc_target_prev = 0.0
        self.previous_curvature = 0.0

        # We'll store multi-step speeds for the next ~10s (33 steps).
        self.planned_speeds = np.zeros(33, dtype=float)

        # Minimum steps we need to do multi-step planning (apex detection).
        self.MIN_STEPS_FOR_APEX = 3

    def reset(self, v_ego: float, curvature: float) -> None:
        self.vtsc_target_prev = v_ego
        self.current_decel = 0.0
        self.previous_curvature = curvature
        self.planned_speeds[:] = v_ego

    def _compute_curvature_array(self, orientation_rate, velocity):
        eps = 1e-6
        return np.array([
            orientation_rate[i] / max(velocity[i], eps)
            for i in range(len(orientation_rate))
        ], dtype=float)

    def _interpolate_to_33(self, arr: np.ndarray) -> np.ndarray:

        """
        Given an array of length m (where 3 <= m < 33),
        linearly interpolate it up to length 33.
        """

        m = len(arr)
        x_src = np.arange(m)
        x_dst = np.linspace(0, m - 1, 33)
        return np.interp(x_dst, x_src, arr)

    def _plan_speed_trajectory(
        self,
        orientation_rate: np.ndarray,
        velocity_pred: np.ndarray,
        init_speed: float,
        turn_aggressiveness: float,
    ) -> np.ndarray:
        """
        Same logic as before, but with an extra margin-based backward pass
        to force earlier slow-down for tighter turns.
        """
        n = len(orientation_rate)  # should be 33 by the time we get here
        dt = DT_MDL

        # 1) Base "safe speeds"
        curvature_array = self._compute_curvature_array(orientation_rate, velocity_pred)
        safe_speeds = np.zeros(n, dtype=float)
        for i in range(n):
            lat_acc_limit = nonlinear_lat_accel(velocity_pred[i], turn_aggressiveness)
            c = max(curvature_array[i], 1e-6)
            safe_speed = math.sqrt(lat_acc_limit / c) if c > 1e-6 else 70.0
            safe_speeds[i] = clip(safe_speed, 0.0, 70.0)

        # 2) "Human-like" apex logic (unchanged)
        apex_idxs = find_apexes(curvature_array)
        for apex_i in apex_idxs:
            apex_speed = safe_speeds[apex_i]
            decel_window = int(max(2, (velocity_pred[apex_i] * 0.12) / dt))
            spool_window = int(max(2, (velocity_pred[apex_i] * 0.07) / dt))

            decel_start = max(0, apex_i - decel_window)
            spool_start = max(0, apex_i - spool_window)
            spool_end   = min(n, apex_i + spool_window + 1)

            # Ramp down to apex
            if spool_start > decel_start:
                v_decel_start = safe_speeds[decel_start]
                steps_decel = spool_start - decel_start
                for idx in range(decel_start, spool_start):
                    f = (idx - decel_start) / float(steps_decel)
                    safe_speeds[idx] = (v_decel_start * (1.0 - f)
                                        + apex_speed * f)

            for idx in range(spool_start, apex_i + 1):
                safe_speeds[idx] = min(safe_speeds[idx], apex_speed)

            # Spool-up from apex
            if spool_end > apex_i:
                steps_spool = spool_end - apex_i
                v_spool_end = safe_speeds[spool_end - 1]
                for idx in range(apex_i, spool_end):
                    f = (idx - apex_i) / float(steps_spool)
                    orig_val = safe_speeds[idx]
                    spool_val = apex_speed * (1.0 - f) + v_spool_end * f
                    safe_speeds[idx] = min(orig_val, spool_val)

        planned = safe_speeds.copy()

        ########################################################
        # 3) MARGIN-BASED BACKWARD PASS (forces earlier slowdown)
        ########################################################
        # Approx 1 second margin, but safely bounded by n-1
        margin_steps = min(n - 1, int(1.0 / dt))
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

        #################################################
        # 4) STANDARD BACKWARD PASS (decel feasibility)
        #################################################
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

        #################################################
        # 5) FORWARD PASS (accel feasibility)
        #################################################
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

    def update(
        self,
        v_ego: float,
        curvature: float,
        turn_aggressiveness: float,
        orientationRate: np.ndarray = None,
        velocityPred: np.ndarray = None
    ) -> float:
        """
        Main update function.
        1) If orientationRate & velocityPred have >= self.MIN_STEPS_FOR_APEX points
           (i.e. >= 3), then we either upsample to 33 or slice the first 33.
        2) If fewer than 3 points, fallback to single-step logic.
        """
        if orientationRate is not None and velocityPred is not None:
            m = len(orientationRate)
            if m == len(velocityPred) and m >= self.MIN_STEPS_FOR_APEX:
                # Upsample or slice to 33
                if 3 <= m < 33:
                    orientationRate_33 = self._interpolate_to_33(orientationRate)
                    velocityPred_33 = self._interpolate_to_33(velocityPred)
                else:
                    orientationRate_33 = orientationRate[:33]
                    velocityPred_33 = velocityPred[:33]

                # Plan with multi-step trajectory
                self.planned_speeds = self._plan_speed_trajectory(
                    orientation_rate=orientationRate_33,
                    velocity_pred=velocityPred_33,
                    init_speed=self.vtsc_target_prev,
                    turn_aggressiveness=turn_aggressiveness
                )
                raw_target = self.planned_speeds[0]
            else:
                # Fallback if not enough data or mismatch
                raw_target = self._single_step_fallback(
                    v_ego, curvature, turn_aggressiveness
                )
        else:
            # Fallback if no data
            raw_target = self._single_step_fallback(
                v_ego, curvature, turn_aggressiveness
            )

        # final symmetrical jerk-limiting
        dt = DT_MDL
        accel_cmd = (raw_target - self.vtsc_target_prev) / dt
        accel_cmd = clip(accel_cmd, -self.MAX_DECEL, self.MAX_DECEL)

        accel_diff = accel_cmd - self.current_decel
        max_delta = self.MAX_JERK * dt
        if accel_diff > max_delta:
            self.current_decel += max_delta
        elif accel_diff < -max_delta:
            self.current_decel -= max_delta
        else:
            self.current_decel = accel_cmd

        jerk_limited_target = self.vtsc_target_prev + self.current_decel * dt

        # Store states
        self.vtsc_target_prev = jerk_limited_target
        self.previous_curvature = curvature

        return jerk_limited_target

    def _single_step_fallback(self, v_ego, curvature, turn_aggressiveness):
        lat_acc = nonlinear_lat_accel(v_ego, turn_aggressiveness)
        v_curvature_ms = clip(math.sqrt(lat_acc / max(curvature, 1e-6)), 0.0, 70.0)
        current_lat_acc = curvature * (v_ego ** 2)

        if current_lat_acc > self.LOW_LAT_ACC:
            if current_lat_acc < self.HIGH_LAT_ACC:
                alpha = 0.1
            else:
                alpha = self.turn_smoothing_alpha
            raw_target = alpha * self.vtsc_target_prev + (1.0 - alpha) * v_curvature_ms
        else:
            alpha = self.reaccel_alpha
            raw_target = alpha * self.vtsc_target_prev + (1.0 - alpha) * v_curvature_ms

        # small apex nudge if curvature is dropping
        if curvature < self.previous_curvature:
            apex_alpha = 0.2
            raw_target = (1.0 - apex_alpha) * raw_target + apex_alpha * v_curvature_ms

        return raw_target
