import math
import numpy as np

# from openpilot.common.realtime import DT_MDL (already included below)
from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import DT_MDL

from openpilot.selfdrive.controls.controlsd import ButtonType
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_UNSET

from openpilot.selfdrive.frogpilot.controls.lib.map_turn_speed_controller import MapTurnSpeedController
from openpilot.selfdrive.frogpilot.controls.lib.speed_limit_controller import SpeedLimitController
from openpilot.selfdrive.frogpilot.frogpilot_variables import CRUISING_SPEED, PLANNER_TIME, params_memory

from openpilot.selfdrive.frogpilot.frogpilot_utilities import calculate_road_curvature

# -------------------------------------------------------------------------
#  Import the dedicated VTSC module
# -------------------------------------------------------------------------
from openpilot.selfdrive.frogpilot.controls.lib.chauffeur_vtsc import VisionTurnSpeedController


class FrogPilotVCruise:
    def __init__(self, FrogPilotPlanner):
        self.frogpilot_planner = FrogPilotPlanner

        # Sub-controllers
        self.mtsc = MapTurnSpeedController()
        self.slc = SpeedLimitController()

        # Force-stop logic
        self.forcing_stop = False
        self.override_force_stop = False
        self.override_slc = False
        self.speed_limit_changed = False

        # Timers
        self.force_stop_timer = 0
        self.override_force_stop_timer = 0
        self.speed_limit_timer = 0

        # Targets
        self.mtsc_target = 0
        self.slc_target = 0
        self.vtsc_target = 0
        self.overridden_speed = 0

        # Speed Limit tracking
        self.previous_speed_limit = 0
        self.tracked_model_length = 0

        # Adjust turning threshold
        self.turn_lat_acc_threshold = 0.3  # was 0.5
        global CRUISING_SPEED
        CRUISING_SPEED = 6.7  # ~15 mph in m/s
        self.turn_smoothing_alpha = 0.3

        # Initialize the Vision Turn Speed Controller
        self.vtsc = VisionTurnSpeedController(
            turn_smoothing_alpha=self.turn_smoothing_alpha,
            reaccel_alpha=0.2,
            low_lat_acc=0.20,
            high_lat_acc=0.40,
            max_decel=2.0,
            max_jerk=1.2
        )

    def update(self, carControl, carState, controlsState,
               frogpilotCarControl, frogpilotCarState, frogpilotNavigation,
               v_cruise, v_ego, frogpilot_toggles):

        # -------------------------------------------------------------
        # Force Stop Logic
        # -------------------------------------------------------------
        from openpilot.common.realtime import DT_MDL

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
        # Map Turn Speed Controller
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
        # Speed Limit Controller (KEY FIX HERE)
        # -------------------------------------------------------------
        if frogpilot_toggles.show_speed_limits or frogpilot_toggles.speed_limit_controller:
            self.slc.update(
                frogpilotCarState.dashboardSpeedLimit,
                controlsState.enabled,
                frogpilotNavigation.navigationSpeedLimitRealtime,
                frogpilotNavigation.mapSpeedLimitRealtime,
                frogpilotNavigation.upcomingSpeedLimitRealtime,
                v_cruise_cluster,
                v_ego,
                frogpilot_toggles
            )
            unconfirmed_slc_target = self.slc.desired_speed_limit

            # Optional user-confirmation logic (unchanged)
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

        # -------------------------------------------------------------
        # Vision Turn Speed Controller
        # -------------------------------------------------------------
        if frogpilot_toggles.vision_turn_controller and v_ego > CRUISING_SPEED and carControl.longActive:
            self.vtsc_target = self.vtsc.update(
                v_ego,
                v_cruise,
                frogpilot_toggles.turn_aggressiveness
            )
        else:
            self.vtsc.reset(v_ego)
            self.vtsc_target = v_cruise if v_cruise != V_CRUISE_UNSET else 0

        # -------------------------------------------------------------
        # Force Standstill / Stop
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
