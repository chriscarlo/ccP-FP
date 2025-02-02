#!/usr/bin/env python3
import math
from openpilot.common.numpy_fast import clip, interp
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
    COMFORT_BRAKE,
    STOP_DISTANCE,
    desired_follow_distance,
    get_T_FOLLOW,
    get_safe_obstacle_distance,
    get_stopped_equivalence_factor
)
from openpilot.selfdrive.frogpilot.frogpilot_variables import CITY_SPEED_LIMIT, CRUISING_SPEED

# Breakpoints for traffic mode: from 0 mph to CITY_SPEED_LIMIT
TRAFFIC_MODE_BP = [0.0, CITY_SPEED_LIMIT]

# --- Local definition of get_jerk_factor ---
# (This helper used to be provided by the MPC module.
#  We now define it locally to preserve backward compatibility.)
def get_jerk_factor(aggressive_jerk_accel, aggressive_jerk_danger, aggressive_jerk_speed,
                    standard_jerk_accel, standard_jerk_danger, standard_jerk_speed,
                    relaxed_jerk_accel, relaxed_jerk_danger, relaxed_jerk_speed,
                    custom_personalities, personality):
    from cereal import log
    if custom_personalities:
        if personality == log.LongitudinalPersonality.relaxed:
            return relaxed_jerk_accel, relaxed_jerk_danger, relaxed_jerk_speed
        elif personality == log.LongitudinalPersonality.standard:
            return standard_jerk_accel, standard_jerk_danger, standard_jerk_speed
        elif personality == log.LongitudinalPersonality.aggressive:
            return aggressive_jerk_accel, aggressive_jerk_danger, aggressive_jerk_speed
        else:
            raise NotImplementedError("Longitudinal personality not supported")
    else:
        return standard_jerk_accel, standard_jerk_danger, standard_jerk_speed

def get_coast_accel(pitch):
    # Returns a coast acceleration based on the vehicle's pitch.
    return np.sin(pitch) * -5.65 - 0.3

class FrogPilotFollowing:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    # State flags
    self.following_lead = False
    self.slower_lead = False

    # Jerk/follow parameters (computed locally, not directly pushed into the MPC)
    self.acceleration_jerk = 0.0
    self.base_acceleration_jerk = 0.0
    self.base_speed_jerk = 0.0
    self.base_danger_jerk = 0.0
    self.speed_jerk = 0.0
    self.danger_jerk = 0.0
    self.t_follow = 1.45  # default value, updated dynamically

    # For external reference (logging, UI, etc.)
    self.desired_follow_distance = 0
    self.safe_obstacle_distance = 0
    self.safe_obstacle_distance_stock = 0
    self.stopped_equivalence_factor = 0

  def update(self, aEgo, controlsState, frogpilotCarState,
             lead_distance, v_ego, v_lead, frogpilot_toggles):
    """
    Computes nominal t_follow and local jerk values based on current state.
    These parameters (such as t_follow, desired follow distance, etc.) are used
    for logging/UI and to inform external modules. The MPC itself handles dynamic
    jerk constraints.
    """
    # -----------------------------------------------------------------------
    # 1) Determine base jerk values and t_follow from toggles/personality
    # -----------------------------------------------------------------------
    if frogpilotCarState.trafficModeActive:
      # Traffic mode: use speed-based interpolation.
      if aEgo >= 0:
        self.base_acceleration_jerk = interp(
          v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_acceleration
        )
        self.base_speed_jerk = interp(
          v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_speed
        )
      else:
        self.base_acceleration_jerk = interp(
          v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_deceleration
        )
        self.base_speed_jerk = interp(
          v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_speed_decrease
        )
      self.base_danger_jerk = interp(
        v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_danger
      )
      self.t_follow = interp(
        v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_follow
      )
    else:
      # Personalities: use get_jerk_factor() and get_T_FOLLOW().
      if aEgo >= 0:
        (self.base_acceleration_jerk,
         self.base_danger_jerk,
         self.base_speed_jerk) = get_jerk_factor(
           frogpilot_toggles.aggressive_jerk_acceleration,
           frogpilot_toggles.aggressive_jerk_danger,
           frogpilot_toggles.aggressive_jerk_speed,
           frogpilot_toggles.standard_jerk_acceleration,
           frogpilot_toggles.standard_jerk_danger,
           frogpilot_toggles.standard_jerk_speed,
           frogpilot_toggles.relaxed_jerk_acceleration,
           frogpilot_toggles.relaxed_jerk_danger,
           frogpilot_toggles.relaxed_jerk_speed,
           frogpilot_toggles.custom_personalities,
           controlsState.personality
         )
      else:
        (self.base_acceleration_jerk,
         self.base_danger_jerk,
         self.base_speed_jerk) = get_jerk_factor(
           frogpilot_toggles.aggressive_jerk_deceleration,
           frogpilot_toggles.aggressive_jerk_danger,
           frogpilot_toggles.aggressive_jerk_speed_decrease,
           frogpilot_toggles.standard_jerk_deceleration,
           frogpilot_toggles.standard_jerk_danger,
           frogpilot_toggles.standard_jerk_speed_decrease,
           frogpilot_toggles.relaxed_jerk_deceleration,
           frogpilot_toggles.relaxed_jerk_danger,
           frogpilot_toggles.relaxed_jerk_speed_decrease,
           frogpilot_toggles.custom_personalities,
           controlsState.personality
         )
      self.t_follow = get_T_FOLLOW(
        frogpilot_toggles.aggressive_follow,
        frogpilot_toggles.standard_follow,
        frogpilot_toggles.relaxed_follow,
        frogpilot_toggles.custom_personalities,
        controlsState.personality
      )
      # Harmonize with planner logic: if the lead is nearly stopped or speeds are very close,
      # adjust t_follow accordingly.
      if v_lead < 0.5:
        self.t_follow = 1.35
      elif abs(v_ego - v_lead) < 1.0:
        self.t_follow = max(self.t_follow - 0.2, 1.0)

    # Store these base jerk values for external reference.
    self.acceleration_jerk = self.base_acceleration_jerk
    self.danger_jerk = self.base_danger_jerk
    self.speed_jerk = self.base_speed_jerk

    # -----------------------------------------------------------------------
    # 2) Determine if we are following a lead.
    # -----------------------------------------------------------------------
    self.following_lead = (
      self.frogpilot_planner.tracking_lead and
      (lead_distance < (self.t_follow + 1.0) * v_ego)
    )

    if self.frogpilot_planner.tracking_lead:
      # Apply additional tweaks to t_follow for a human-like following style.
      self.update_follow_values(lead_distance, v_ego, v_lead, frogpilot_toggles)

      # Update references for external modules.
      self.desired_follow_distance = int(desired_follow_distance(v_ego, v_lead, self.t_follow))
      self.safe_obstacle_distance = int(get_safe_obstacle_distance(v_ego, self.t_follow))
      self.safe_obstacle_distance_stock = self.safe_obstacle_distance  # Placeholder for alternate logic.
      self.stopped_equivalence_factor = int(get_stopped_equivalence_factor(v_lead))
    else:
      self.desired_follow_distance = 0
      self.safe_obstacle_distance = 0
      self.safe_obstacle_distance_stock = 0
      self.stopped_equivalence_factor = 0

  def update_follow_values(self, lead_distance, v_ego, v_lead, frogpilot_toggles):
    """
    Provides mild additional adjustments to t_follow to emulate a natural,
    human-like approach. This logic avoids abrupt changes when a new lead cuts in.
    """
    # If the lead is nearly stopped, force a fixed t_follow.
    if v_lead < 0.5:
      self.t_follow = 1.35
      return

    # If the lead is faster than ego, gently reduce t_follow.
    if frogpilot_toggles.human_following and v_lead > v_ego:
      acceleration_offset = clip((v_lead - v_ego) - COMFORT_BRAKE, 0.9, 1.1)
      self.t_follow *= (1.0 / acceleration_offset)

    # If the lead is slower and we're above cruising speed,
    # gently increase t_follow to allow the gap to open gradually.
    if ((frogpilot_toggles.conditional_slower_lead or frogpilot_toggles.human_following)
        and v_lead < v_ego and v_ego > CRUISING_SPEED):
      braking_offset = clip((v_ego - v_lead) - COMFORT_BRAKE, 0.9, 1.1)
      if frogpilot_toggles.human_following:
        self.t_follow *= braking_offset
      self.slower_lead = (braking_offset > 1.0)
