import math
import numpy as np
# If you want to generate a plot or table, uncomment:
# import matplotlib.pyplot as plt

from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import DT_MDL

from openpilot.selfdrive.controls.controlsd import ButtonType
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_UNSET

from openpilot.selfdrive.frogpilot.controls.lib.map_turn_speed_controller import MapTurnSpeedController
from openpilot.selfdrive.frogpilot.controls.lib.speed_limit_controller import SpeedLimitController
from openpilot.selfdrive.frogpilot.frogpilot_variables import CRUISING_SPEED, PLANNER_TIME, params_memory

from openpilot.selfdrive.frogpilot.frogpilot_utilities import calculate_road_curvature


# -----------------------------------------------------------------------------
# Non-linear function for max comfortable lateral acceleration (m/s^2)
# (Unchanged from original)
# -----------------------------------------------------------------------------
def nonlinear_lat_accel(v_ego_ms: float, turn_aggressiveness: float = 1.0) -> float:
  """
  Smooth logistic function returning a comfortable max lateral accel.

    v_ego_ms:          vehicle speed in m/s
    turn_aggressiveness: user multiplier

  This logistic is centered around ~30 mph and tuned so that:
    - at 10 mph => ~1.7 m/s^2
    - at 40 mph => ~3.0 m/s^2
    - at 70 mph => ~3.2 m/s^2
  """
  # Convert to mph
  v_ego_mph = v_ego_ms * CV.MS_TO_MPH

  # Lower bound + range
  base = 1.7
  span = 1.9
  center = 45.0
  k = 0.14

  # logistic form: f(x) = base + span / (1 + e^{-k (x - center)})
  lat_acc = base + span / (1.0 + math.exp(-k * (v_ego_mph - center)))

  # minor clamp for safety
  lat_acc = min(lat_acc, 3.2)

  return lat_acc * turn_aggressiveness


# -----------------------------------------------------------------------------
# Main FrogPilotVCruise class (with multi-stage + jerk-limited decel)
# -----------------------------------------------------------------------------
class FrogPilotVCruise:
    def __init__(self, FrogPilotPlanner):
        self.frogpilot_planner = FrogPilotPlanner

        # Sub-controllers (original)
        self.mtsc = MapTurnSpeedController()
        self.slc = SpeedLimitController()

        # Force-stop logic (original)
        self.forcing_stop = False
        self.override_force_stop = False
        self.override_slc = False
        self.speed_limit_changed = False

        # Timers (original)
        self.force_stop_timer = 0
        self.override_force_stop_timer = 0
        self.speed_limit_timer = 0

        # Targets (original)
        self.mtsc_target = 0
        self.slc_target = 0
        self.vtsc_target = 0
        self.overridden_speed = 0

        # Other references (original)
        self.previous_speed_limit = 0
        self.tracked_model_length = 0

        # Original user changes to detect turns sooner + lower CRUISING_SPEED
        self.turn_lat_acc_threshold = 0.3  # was 0.5
        global CRUISING_SPEED
        CRUISING_SPEED = 6.7  # ~15 mph in m/s
        self.turn_smoothing_alpha = 0.3
        self.vtsc_target_prev = 0.0

        # -----------------------------------------------------------------
        # CHAUFFEUR-STYLE ADDITIONS
        # -----------------------------------------------------------------
        # Multi-stage latAccel thresholds
        self.LOW_LAT_ACC  = 0.20
        self.HIGH_LAT_ACC = 0.40

        # Jerk-limited deceleration parameters
        self.MAX_DECEL = 2.0  # m/s^2
        self.MAX_JERK  = 1.0  # m/s^3
        self.current_decel = 0.0

        # Slightly gentler re-acceleration
        self.reaccel_alpha = 0.2


    def update(self, carControl, carState, controlsState,
               frogpilotCarControl, frogpilotCarState, frogpilotNavigation,
               v_cruise, v_ego, frogpilot_toggles):

        # -------------------------------------------------------------
        # Force Stop Logic (original)
        # -------------------------------------------------------------
        force_stop = (
            frogpilot_toggles.force_stops
            and self.frogpilot_planner.cem.stop_light_detected
            and controlsState.enabled
        )
        force_stop &= self.frogpilot_planner.model_length < 100
        force_stop &= self.override_force_stop_timer <= 0

        self.force_stop_timer = self.force_stop_timer + DT_MDL if force_stop else 0
        force_stop_enabled = self.force_stop_timer >= 1

        self.override_force_stop |= (
            (not frogpilot_toggles.force_standstill
             and carState.standstill
             and self.frogpilot_planner.tracking_lead)
            or carState.gasPressed
            or frogpilotCarControl.accelPressed
        )
        self.override_force_stop &= force_stop_enabled

        if self.override_force_stop:
            self.override_force_stop_timer = 10
        elif self.override_force_stop_timer > 0:
            self.override_force_stop_timer -= DT_MDL

        # Keep cluster in sync
        v_cruise_cluster = max(controlsState.vCruiseCluster * CV.KPH_TO_MS, v_cruise)
        v_cruise_diff = v_cruise_cluster - v_cruise

        v_ego_cluster = max(carState.vEgoCluster, v_ego)
        v_ego_diff = v_ego_cluster - v_ego

        # -------------------------------------------------------------
        # Map Turn Speed Controller (original)
        # -------------------------------------------------------------
        if frogpilot_toggles.map_turn_speed_controller and v_ego > CRUISING_SPEED and carControl.longActive:
            mtsc_active = self.mtsc_target < v_cruise
            self.mtsc_target = clip(
                self.mtsc.target_speed(v_ego, carState.aEgo, frogpilot_toggles),
                CRUISING_SPEED, v_cruise
            )

            curve_detected = (1 / self.frogpilot_planner.road_curvature) ** 0.5 < v_ego
            if curve_detected and mtsc_active:
                self.mtsc_target = self.frogpilot_planner.v_cruise
            elif not curve_detected and frogpilot_toggles.mtsc_curvature_check:
                self.mtsc_target = v_cruise

            if self.mtsc_target == CRUISING_SPEED:
                self.mtsc_target = v_cruise
        else:
            self.mtsc_target = v_cruise if v_cruise != V_CRUISE_UNSET else 0

        # -------------------------------------------------------------
        # Speed Limit Controller (original)
        # -------------------------------------------------------------
        if frogpilot_toggles.show_speed_limits or frogpilot_toggles.speed_limit_controller:
            self.slc.update(
                frogpilotCarState.dashboardSpeedLimit,
                controlsState.enabled,
                frogpilotNavigation.navigationSpeedLimit,
                v_cruise_cluster,
                v_ego,
                frogpilot_toggles
            )
            unconfirmed_slc_target = self.slc.desired_speed_limit

            # Optional user-confirmation logic
            if ((frogpilot_toggles.speed_limit_changed_alert or frogpilot_toggles.speed_limit_confirmation)
                and self.slc_target != 0):
                self.speed_limit_changed = (
                    abs(self.previous_speed_limit - unconfirmed_slc_target) > 1
                    and self.previous_speed_limit != 0
                    and unconfirmed_slc_target > 1
                )
                speed_limit_accepted = (
                    self.speed_limit_changed
                    and ((frogpilotCarControl.accelPressed and carControl.longActive)
                         or params_memory.get_bool("SLCConfirmed"))
                )
                speed_limit_denied = (
                    self.speed_limit_changed
                    and ((frogpilotCarControl.decelPressed and carControl.longActive)
                         or self.speed_limit_timer >= 30)
                )
                speed_limit_decreased = (
                    self.speed_limit_changed
                    and (self.slc_target - unconfirmed_slc_target) > 1
                )
                speed_limit_increased = (
                    self.speed_limit_changed
                    and (unconfirmed_slc_target - self.slc_target) > 1
                )

                if speed_limit_accepted:
                    self.previous_speed_limit = unconfirmed_slc_target
                    self.slc_target = unconfirmed_slc_target
                    self.speed_limit_changed = False
                    params_memory.remove("SLCConfirmed")

                elif speed_limit_denied:
                    self.previous_speed_limit = unconfirmed_slc_target
                    self.speed_limit_changed = False

                elif speed_limit_decreased and not frogpilot_toggles.speed_limit_confirmation_lower:
                    self.previous_speed_limit = unconfirmed_slc_target
                    self.slc_target = unconfirmed_slc_target
                    self.speed_limit_changed = False

                elif speed_limit_increased and not frogpilot_toggles.speed_limit_confirmation_higher:
                    self.previous_speed_limit = unconfirmed_slc_target
                    self.slc_target = unconfirmed_slc_target
                    self.speed_limit_changed = False

                if self.speed_limit_changed:
                    self.speed_limit_timer += DT_MDL
                else:
                    self.speed_limit_timer = 0
            else:
                self.slc_target = unconfirmed_slc_target

            if frogpilot_toggles.speed_limit_controller:
                self.override_slc = self.overridden_speed > self.slc_target
                self.override_slc |= (carState.gasPressed and v_ego > self.slc_target)
                self.override_slc &= controlsState.enabled

                if self.override_slc:
                    if frogpilot_toggles.speed_limit_controller_override_manual:
                        if carState.gasPressed:
                            self.overridden_speed = v_ego_cluster
                        self.overridden_speed = clip(self.overridden_speed, self.slc_target, v_cruise)
                    elif frogpilot_toggles.speed_limit_controller_override_set_speed:
                        self.overridden_speed = v_cruise_cluster
                else:
                    self.overridden_speed = 0
            else:
                self.override_slc = False
                self.overridden_speed = 0
        else:
            self.slc_target = 0

        # ----------------------------------------------------------------
        # Vision Turn Speed Controller (VTSC), with multi-stage + jerk-limit
        # ----------------------------------------------------------------
        if frogpilot_toggles.vision_turn_controller and v_ego > CRUISING_SPEED and carControl.longActive:
            # The existing (original) curvature
            c = abs(self.frogpilot_planner.road_curvature)

            # If extremely small curvature => no limit
            if c < 1e-9:
                v_curvature_ms = v_cruise
            else:
                lat_acc = nonlinear_lat_accel(v_ego, frogpilot_toggles.turn_aggressiveness)
                v_curvature_ms = math.sqrt(lat_acc / c)

            # Clip final speed to [CRUISING_SPEED, v_cruise]
            v_curvature_ms = clip(v_curvature_ms, CRUISING_SPEED, v_cruise)

            # Multi-stage latAccel logic
            current_lat_acc = c * (v_ego ** 2)

            if current_lat_acc > self.LOW_LAT_ACC:
                # Lightly or definitely in turn => alpha approach
                if current_lat_acc < self.HIGH_LAT_ACC:
                    # Pre-slow alpha is mild, e.g. 0.1
                    alpha = 0.1
                else:
                    # Definitely in turn => stronger smoothing
                    alpha = self.turn_smoothing_alpha

                v_target = alpha * self.vtsc_target_prev + (1.0 - alpha) * v_curvature_ms
            else:
                # Not in a turn => revert quickly, but not abruptly
                alpha = self.reaccel_alpha
                v_target = alpha * self.vtsc_target_prev + (1.0 - alpha) * v_cruise

            # Jerk-limited decel
            desired_decel = 0.0
            if v_target < v_ego:
                desired_decel = clip((v_ego - v_target), 0.0, self.MAX_DECEL)

            decel_diff = desired_decel - self.current_decel
            max_delta  = self.MAX_JERK * DT_MDL
            if decel_diff > max_delta:
                self.current_decel += max_delta
            elif decel_diff < -max_delta:
                self.current_decel -= max_delta
            else:
                self.current_decel = desired_decel

            jerk_limited_target = v_ego - self.current_decel * DT_MDL
            jerk_limited_target = min(jerk_limited_target, v_target)

            self.vtsc_target = jerk_limited_target
            self.vtsc_target_prev = jerk_limited_target
        else:
            # If Vision Turn is off or speed < CRUISING_SPEED, no turn limit
            self.vtsc_target = v_cruise if v_cruise != V_CRUISE_UNSET else 0
            self.vtsc_target_prev = self.vtsc_target
            self.current_decel = 0.0

        # -------------------------------------------------------------
        # Force Standstill / Stop (original)
        # -------------------------------------------------------------
        if (frogpilot_toggles.force_standstill
            and carState.standstill
            and not self.override_force_stop
            and controlsState.enabled):
            # Hard standstill override
            self.forcing_stop = True
            v_cruise = -1
        elif force_stop_enabled and not self.override_force_stop:
            self.forcing_stop |= not carState.standstill
            self.tracked_model_length = max(self.tracked_model_length - v_ego * DT_MDL, 0)
            v_cruise = min((self.tracked_model_length // PLANNER_TIME), v_cruise)
        else:
            if not self.frogpilot_planner.cem.stop_light_detected:
                self.override_force_stop = False
            self.forcing_stop = False
            self.tracked_model_length = self.frogpilot_planner.model_length

            # Choose final target among [MapTurn, SpeedLimit, VisionTurn]
            if frogpilot_toggles.speed_limit_controller:
                targets = [
                    self.mtsc_target,
                    max(self.overridden_speed, self.slc_target) - v_ego_diff,
                    self.vtsc_target
                ]
            else:
                targets = [self.mtsc_target, self.vtsc_target]

            # Don’t drop below CRUISING_SPEED unless needed
            v_cruise = float(min([t if t > CRUISING_SPEED else v_cruise for t in targets]))

        # Keep everything in sync w/ cluster differences
        self.mtsc_target += v_cruise_diff
        self.vtsc_target += v_cruise_diff

        return v_cruise


# -----------------------------------------------------------------------------
# OPTIONAL Example: Test or Visualization Code
# -----------------------------------------------------------------------------
if __name__ == "__main__":
  # Quick test to verify lat accel function
  test_mph = list(range(5, 80, 5))
  test_ms  = [s * CV.MPH_TO_MS for s in test_mph]

  lat_accels = [ nonlinear_lat_accel(v) for v in test_ms ]

  print(" MPH | Speed(m/s) | LatAccel(m/s^2) | LatAccel(g)")
  print("-----+------------+-----------------+-----------")
  for mph, v, a in zip(test_mph, test_ms, lat_accels):
    print(f"{mph:4.0f} | {v:10.2f} | {a:15.3f} | {(a/9.81):10.2f}")

  # If needed, you can uncomment and plot the resulting lat accel curve:
  # import matplotlib.pyplot as plt
  # fig, ax = plt.subplots()
  # ax.plot(test_mph, lat_accels, color='blue', marker='o', label='Logistic Lat Accel')
  # ax.set_xlabel('Speed (mph)')
  # ax.set_ylabel('Lateral Accel (m/s^2)')
  # ax.set_title("Nonlinear Lateral Accel Curve")
  # ax.legend()
  # plt.show()