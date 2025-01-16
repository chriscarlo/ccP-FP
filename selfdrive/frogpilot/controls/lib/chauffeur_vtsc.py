import math
import numpy as np

from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import DT_MDL

def nonlinear_lat_accel(v_ego_ms: float, turn_aggressiveness: float = 1.0) -> float:
  """
  Smooth logistic function returning a comfortable max lateral accel.

  This logistic is centered around ~30 mph and tuned so that:
    - at 10 mph => ~1.7 m/s^2
    - at 40 mph => ~3.0 m/s^2
    - at 70 mph => ~3.2 m/s^2

  v_ego_ms: vehicle speed in m/s
  turn_aggressiveness: user multiplier, e.g. 1.0 for default
  """
  v_ego_mph = v_ego_ms * CV.MS_TO_MPH

  base = 1.7
  span = 1.9
  center = 45.0
  k = 0.14

  lat_acc = base + span / (1.0 + math.exp(-k * (v_ego_mph - center)))
  lat_acc = min(lat_acc, 3.2)  # clamp for safety
  return lat_acc * turn_aggressiveness

def find_apexes(curvature: np.ndarray, threshold: float = 1e-4) -> list:
    """
    Find indices of "apexes" (local maxima) in the curvature array.
    An apex is defined where:
      curvature[i] > curvature[i-1]
      AND curvature[i] >= curvature[i+1]
      AND curvature[i] > threshold

    Returns a list of integer indices that satisfy this condition.
    """
    apex_indices = []
    for i in range(1, len(curvature) - 1):
        if (curvature[i] > curvature[i - 1]) and (curvature[i] >= curvature[i + 1]) \
           and (curvature[i] > threshold):
            apex_indices.append(i)
    return apex_indices

class VisionTurnSpeedController:
    """
    A multi-step Vision Turn Speed Controller (VTSC) for Chauffeur, computing a
    10-second speed plan using a forward-backward pass and apex detection.
    This design aims to create a professional, "chauffeur-like" experience:
      - Smooth deceleration into corners
      - Gentle acceleration out of apexes
      - Handling of multiple apexes gracefully
      - Maintaining jerk-limited transitions
    """

    def __init__(
        self,
        turn_smoothing_alpha=0.3,
        reaccel_alpha=0.2,
        low_lat_acc=0.20,
        high_lat_acc=0.40,
        max_decel=2.0,
        max_jerk=1.2,
    ):
        self.turn_smoothing_alpha = turn_smoothing_alpha
        self.reaccel_alpha = reaccel_alpha
        self.LOW_LAT_ACC = low_lat_acc
        self.HIGH_LAT_ACC = high_lat_acc
        self.MAX_DECEL = max_decel
        self.MAX_JERK = max_jerk

        # Internal states
        self.vtsc_target_prev = 0.0
        self.current_decel = 0.0
        self.previous_curvature = 0.0

        # Store the planned speeds for the next ~10s (33 model steps)
        self.planned_speeds = np.zeros(33, dtype=float)

    def reset(self, v_ego: float, curvature: float) -> None:
        """
        Resets the internal states for a "fresh" cycle when VTSC is turned off
        or when speed is below cruising threshold.
        """
        self.vtsc_target_prev = v_ego
        self.current_decel = 0.0
        self.previous_curvature = curvature
        self.planned_speeds[:] = v_ego

    def _compute_curvature_array(self, orientation_rate, velocity):
        """
        Compute an array of curvature from orientationRate and velocity:
          curvature[i] = orientation_rate[i] / max(velocity[i], 1e-6)
        """
        eps = 1e-6
        return np.array([
            orientation_rate[i] / max(velocity[i], eps)
            for i in range(len(orientation_rate))
        ], dtype=float)

    def _plan_speed_trajectory(
        self,
        orientation_rate: np.ndarray,
        velocity_pred: np.ndarray,
        v_ego: float,
        turn_aggressiveness: float,
    ) -> np.ndarray:
        """
        Generate a jerk-limited speed profile over the 33 predicted timesteps.
         - find apexes (local maxima of curvature)
         - forward pass to accelerate/decelerate
         - small "apex-out" acceleration factor if next apex is far away
         - backward pass to ensure we can slow for upcoming apexes
        """
        n = len(orientation_rate)  # should be 33
        dt = DT_MDL  # nominal model step
        planned_speeds = np.zeros(n, dtype=float)

        # 1) Compute curvature array and initial "safe speeds" from lat_acc constraints
        curvature_array = self._compute_curvature_array(orientation_rate, velocity_pred)
        for i in range(n):
            lat_acc_limit = nonlinear_lat_accel(velocity_pred[i], turn_aggressiveness)
            c = max(curvature_array[i], 1e-6)
            safe_speed = math.sqrt(lat_acc_limit / c) if c > 1e-6 else 70.0
            planned_speeds[i] = clip(safe_speed, 0.0, 70.0)

        # 2) Identify apex indices in the horizon
        apex_indices = find_apexes(curvature_array)
        # Example: [5, 12, 27] if there are multiple apexes

        # 3) Forward pass
        #
        # Start from the current speed, ensure jerk-limited acceleration,
        # but also add a small "apex-out" acceleration if the next apex is far.

        planned_speeds[0] = min(planned_speeds[0], v_ego)
        accel_cmd = 0.0

        # We'll keep track of where we are relative to the next apex
        apex_ptr = 0

        for i in range(1, n):
            v_prev = planned_speeds[i - 1]

            # If we've passed the apex, increment apex_ptr
            while apex_ptr < len(apex_indices) and i > apex_indices[apex_ptr]:
                apex_ptr += 1

            # Distance to next apex (in indices); if next apex is soon, be cautious
            if apex_ptr < len(apex_indices):
                steps_to_next_apex = apex_indices[apex_ptr] - i
            else:
                steps_to_next_apex = 9999  # no more apexes

            # baseline desired accel
            desired_acc = (planned_speeds[i] - v_prev) / dt

            # "intelligent" apex-out factor:
            # if the next apex is far away => accelerate more
            # if near => accelerate less
            if steps_to_next_apex > 10:
                desired_acc *= 1.2
            elif steps_to_next_apex < 4:
                desired_acc *= 0.7

            # jerk-limit from current accel_cmd
            accel_diff = desired_acc - accel_cmd
            max_delta = self.MAX_JERK * dt
            if accel_diff > max_delta:
                accel_cmd += max_delta
            elif accel_diff < -max_delta:
                accel_cmd -= max_delta
            else:
                accel_cmd = desired_acc

            # clamp and apply
            accel_cmd = clip(accel_cmd, -self.MAX_DECEL, self.MAX_DECEL)
            planned_speeds[i] = v_prev + accel_cmd * dt

        # 4) Backward pass
        #
        # Ensure we can still slow down for upcoming apexes/curves. This step
        # "pulls down" speeds from the end backward, guaranteeing feasible
        # deceleration if a tight turn is imminent.

        accel_cmd = 0.0
        for i in range(n - 2, -1, -1):
            v_next = planned_speeds[i + 1]
            desired_acc = (planned_speeds[i] - v_next) / dt

            accel_diff = desired_acc - accel_cmd
            max_delta = self.MAX_JERK * dt
            if accel_diff > max_delta:
                accel_cmd += max_delta
            elif accel_diff < -max_delta:
                accel_cmd -= max_delta
            else:
                accel_cmd = desired_acc

            accel_cmd = clip(accel_cmd, -self.MAX_DECEL, self.MAX_DECEL)
            feasible_speed = v_next + accel_cmd * dt
            planned_speeds[i] = min(planned_speeds[i], feasible_speed)

        return planned_speeds

    def update(
        self,
        v_ego: float,
        curvature: float,
        turn_aggressiveness: float,
        orientationRate: np.ndarray = None,
        velocityPred: np.ndarray = None
    ) -> float:
        """
        Main update function:
          1) If we have full 33-step horizon data => do multi-step plan
          2) Otherwise => fallback to single-step logic
          3) Return a single jerk-limited speed target (for immediate control)
        """
        # 1) If we have model predictions, run the multi-step plan
        if (orientationRate is not None and velocityPred is not None
                and len(orientationRate) == 33 and len(velocityPred) == 33):
            self.planned_speeds = self._plan_speed_trajectory(
                orientationRate,
                velocityPred,
                v_ego,
                turn_aggressiveness
            )
            raw_target = self.planned_speeds[0]

        else:
            # 2) Fallback single-step logic (for backward compatibility)
            lat_acc = nonlinear_lat_accel(v_ego, turn_aggressiveness)
            v_curvature_ms = clip(
                math.sqrt(lat_acc / max(curvature, 1e-6)),
                0.0,
                70.0
            )
            current_lat_acc = curvature * (v_ego ** 2)

            # Multi-stage logic
            if current_lat_acc > self.LOW_LAT_ACC:
                if current_lat_acc < self.HIGH_LAT_ACC:
                    alpha = 0.1
                else:
                    alpha = self.turn_smoothing_alpha
                raw_target = alpha * self.vtsc_target_prev + (1.0 - alpha) * v_curvature_ms
            else:
                alpha = self.reaccel_alpha
                raw_target = alpha * self.vtsc_target_prev + (1.0 - alpha) * v_curvature_ms

            # minor apex nudge if curvature is dropping
            if curvature < self.previous_curvature:
                apex_alpha = 0.2
                raw_target = (1.0 - apex_alpha) * raw_target + apex_alpha * v_curvature_ms

        # 3) Apply symmetrical jerk-limiting to the immediate target
        dt = DT_MDL
        accel_cmd = (raw_target - v_ego) / dt
        accel_cmd = clip(accel_cmd, -self.MAX_DECEL, self.MAX_DECEL)

        accel_diff = accel_cmd - self.current_decel
        max_delta = self.MAX_JERK * dt
        if accel_diff > max_delta:
            self.current_decel += max_delta
        elif accel_diff < -max_delta:
            self.current_decel -= max_delta
        else:
            self.current_decel = accel_cmd

        jerk_limited_target = v_ego + self.current_decel * dt

        # Update internal states
        self.vtsc_target_prev = jerk_limited_target
        self.previous_curvature = curvature

        # Return the single float for immediate control usage
        return jerk_limited_target
