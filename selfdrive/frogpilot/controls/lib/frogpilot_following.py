#!/usr/bin/env python3
import math
from openpilot.common.numpy_fast import clip, interp
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
    COMFORT_BRAKE,
    STOP_DISTANCE,
    desired_follow_distance,
    get_jerk_factor,
    get_T_FOLLOW,
    get_safe_obstacle_distance,
    get_stopped_equivalence_factor
)
from openpilot.selfdrive.frogpilot.frogpilot_variables import CITY_SPEED_LIMIT, CRUISING_SPEED

TRAFFIC_MODE_BP = [0.0, CITY_SPEED_LIMIT]

class FrogPilotFollowing:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    # State flags
    self.following_lead = False
    self.slower_lead = False

    # Jerk/follow parameters (just stored locally, not pushed into MPC)
    self.acceleration_jerk = 0.0
    self.base_acceleration_jerk = 0.0
    self.base_speed_jerk = 0.0
    self.base_danger_jerk = 0.0
    self.speed_jerk = 0.0
    self.danger_jerk = 0.0
    self.t_follow = 1.45  # default, updated dynamically

    # For reference by external code (logging, UI, etc.)
    self.desired_follow_distance = 0
    self.safe_obstacle_distance = 0
    self.stopped_equivalence_factor = 0

  def update(self, aEgo, controlsState, frogpilotCarState,
             lead_distance, v_ego, v_lead, frogpilot_toggles):
    """
    Picks a nominal t_follow and local jerk variables. The actual dynamic jerk
    constraints are handled by the MPC (we do NOT call mpc.set_weights() here).
    This script also updates safe_obstacle_distance and stopped_equivalence_factor
    for any other code that references them.
    """
    # -----------------------------------------------------------------------
    # 1) Determine the base jerk & t_follow from toggles/personality
    # -----------------------------------------------------------------------
    if frogpilotCarState.trafficModeActive:
      # Traffic mode => speed-based interpolation for jerk & t_follow
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
      # Personalities => get_jerk_factor() + get_T_FOLLOW()
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

    # Store them for reference only
    self.acceleration_jerk = self.base_acceleration_jerk
    self.danger_jerk = self.base_danger_jerk
    self.speed_jerk = self.base_speed_jerk

    # -----------------------------------------------------------------------
    # 2) Are we following a lead? If so, possibly do a mild tweak to t_follow
    # -----------------------------------------------------------------------
    self.following_lead = (
      self.frogpilot_planner.tracking_lead and
      (lead_distance < (self.t_follow + 1.0) * v_ego)
    )

    if self.frogpilot_planner.tracking_lead:
      # Optionally do small local tweaks for “human_following”
      self.update_follow_values(lead_distance, v_ego, v_lead, frogpilot_toggles)

      # Provide references used by external code
      self.desired_follow_distance = int(desired_follow_distance(v_ego, v_lead, self.t_follow))
      self.safe_obstacle_distance = int(get_safe_obstacle_distance(v_ego, self.t_follow))
      self.stopped_equivalence_factor = int(get_stopped_equivalence_factor(v_lead))
    else:
      self.desired_follow_distance = 0
      self.safe_obstacle_distance = 0
      self.stopped_equivalence_factor = 0

  def update_follow_values(self, lead_distance, v_ego, v_lead, frogpilot_toggles):
    """
    Mild additional logic for faster/slower lead, but do NOT override the MPC’s
    jerk or constraints. Just gently tweak t_follow to emulate 'human' approach.
    """
    # Example: if we want to slightly reduce follow distance for a faster lead
    if frogpilot_toggles.human_following and v_lead > v_ego:
      # keep any big multipliers smaller than the old script to avoid oscillation
      distance_factor = max(lead_distance - (v_ego * self.t_follow), 1.0)
      # a tiny offset factor: e.g. clamp to ±10%
      acceleration_offset = clip(
        (v_lead - v_ego) - COMFORT_BRAKE,
        0.9, 1.1
      )
      # reduce t_follow just a bit if lead is significantly faster
      self.t_follow *= (1.0 / acceleration_offset)

    # If lead is slower and we’re above cruising speed
    if ((frogpilot_toggles.conditional_slower_lead or frogpilot_toggles.human_following)
        and v_lead < v_ego > CRUISING_SPEED):

      braking_offset = clip(
        (v_ego - v_lead) - COMFORT_BRAKE,
        0.9, 1.1
      )
      if frogpilot_toggles.human_following:
        self.t_follow *= braking_offset

      self.slower_lead = (braking_offset > 1.0)