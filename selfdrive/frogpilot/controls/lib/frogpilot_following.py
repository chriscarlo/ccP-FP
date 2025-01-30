#!/usr/bin/env python3
import math
from openpilot.common.numpy_fast import clip, interp
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
    COMFORT_BRAKE, STOP_DISTANCE,
    desired_follow_distance, get_jerk_factor, get_T_FOLLOW
)
from openpilot.selfdrive.frogpilot.frogpilot_variables import CITY_SPEED_LIMIT, CRUISING_SPEED

TRAFFIC_MODE_BP = [0., CITY_SPEED_LIMIT]

class FrogPilotFollowing:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    # Tracking flags
    self.following_lead = False
    self.slower_lead = False

    # Jerk parameters (for reference/logging; we do *not* push them to MPC)
    self.acceleration_jerk = 0.0
    self.base_acceleration_jerk = 0.0
    self.base_speed_jerk = 0.0
    self.base_danger_jerk = 0.0
    self.speed_jerk = 0.0
    self.danger_jerk = 0.0

    # Following distance
    self.t_follow = 1.45  # Just a default placeholder
    self.desired_follow_distance = 0.0

  def update(self, aEgo, controlsState, frogpilotCarState, lead_distance, v_ego, v_lead, frogpilot_toggles):
    """
    Main update to pick a 'nominal' t_follow and store local jerk values.
    This script no longer modifies the MPC solver directly. The MPC's own
    dynamic cost logic handles detailed jerk constraints and lead approach.
    """
    # -------------------------------------------------------------
    # 1) Determine base jerk & T_FOLLOW from toggles or personalities
    # -------------------------------------------------------------
    if frogpilotCarState.trafficModeActive:
      # Traffic mode => speed-based interpolation
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
      # Personalities => get_jerk_factor & get_T_FOLLOW
      if aEgo >= 0:
        self.base_acceleration_jerk, self.base_danger_jerk, self.base_speed_jerk = get_jerk_factor(
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
        self.base_acceleration_jerk, self.base_danger_jerk, self.base_speed_jerk = get_jerk_factor(
          frogpilot_toggles.aggressive_jerk_deceleration,
          frogpilot_toggles.aggressive_jerk_danger,
          frogpilot_toggles.aggressive_jerk_speed_decrease,
          frogpilot_toggles.standard_jerk_deceleration,
          frogpilot_toggles.standard_jerk_danger,
          frogpilot_toggles.standard_jerk_speed_decrease,
          frogpilot_toggles.relaxed_jerk_deceleration,
          frogpilot_toggles.relaxed_jerk_danger,
          frogpilot_toggles.relaxed_jerk_speed_deceleration,
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

    # Copy them to local variables, but DO NOT push them to MPC
    self.acceleration_jerk = self.base_acceleration_jerk
    self.danger_jerk = self.base_danger_jerk
    self.speed_jerk = self.base_speed_jerk

    # -------------------------------------------------------------
    # 2) Lead detection
    # -------------------------------------------------------------
    self.following_lead = (
      self.frogpilot_planner.tracking_lead
      and lead_distance < (self.t_follow + 1.0) * v_ego
    )

    if self.frogpilot_planner.tracking_lead:
      # Possibly do small local adjustments for "human_following"
      self.update_follow_values(lead_distance, v_ego, v_lead, frogpilot_toggles)

      # Calculate a reference distance just for logging/UI
      self.desired_follow_distance = int(
        desired_follow_distance(v_ego, v_lead, self.t_follow)
      )
    else:
      self.desired_follow_distance = 0

  def update_follow_values(self, lead_distance, v_ego, v_lead, frogpilot_toggles):
    """
    If you want small adjustments to T_FOLLOW for 'human' or 'conditional' logic,
    do them gently here. Avoid big changes each frame, or you may still get
    see-sawing. And do NOT call mpc.set_weights() or manipulate solver params
    directly. Let the MPC handle jerk constraints internally.
    """

    # Example: if "human_following" is on and lead is faster, reduce t_follow a bit
    if frogpilot_toggles.human_following and v_lead > v_ego:
      # Provide some small, mild factor. Avoid huge changes to t_follow every frame.
      # For instance, clamp to a 10% reduction at most:
      distance_factor = max(lead_distance - (v_ego * self.t_follow), 1.0)
      # This is the old approach, but we reduce how strong it is:
      acceleration_offset = clip(
        (v_lead - v_ego) - COMFORT_BRAKE,   # was (v_lead - v_ego)*standstill_offset - COMFORT_BRAKE
        0.9, 1.1  # keep within ±10%
      )
      self.t_follow *= 1.0 / acceleration_offset

    # Similarly, if lead is slower and we're above cruising speed
    if (frogpilot_toggles.conditional_slower_lead or frogpilot_toggles.human_following) \
       and v_lead < v_ego > CRUISING_SPEED:
      # Tiny nudge to t_follow, not large leaps
      braking_offset = clip(
        (v_ego - v_lead) - COMFORT_BRAKE,
        0.9, 1.1
      )
      # If 'human_following' is true, you might slightly increase t_follow
      if frogpilot_toggles.human_following:
        self.t_follow *= braking_offset
      self.slower_lead = (braking_offset > 1.0)
