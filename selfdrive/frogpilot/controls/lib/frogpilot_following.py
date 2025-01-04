import math
from openpilot.common.numpy_fast import clip, interp
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
    COMFORT_BRAKE,
    STOP_DISTANCE,
    get_jerk_factor,
    get_safe_obstacle_distance,
    get_stopped_equivalence_factor,
    get_T_FOLLOW,
)
from openpilot.selfdrive.frogpilot.frogpilot_variables import CITY_SPEED_LIMIT, CRUISING_SPEED

TRAFFIC_MODE_BP = [0.0, CITY_SPEED_LIMIT]

def logistic_interpolate(x, x_min, x_max, y_min, y_max, slope=1.0):
  """
  Logistic-based interpolation between (x_min -> y_min) and (x_max -> y_max).
  - 'slope' controls how quickly we transition near the midpoint.
  - This is used here strictly to scale jerk at different speeds.
  """
  # Avoid degenerate slopes
  slope = max(abs(slope), 1e-6)

  # Clamp x to [x_min, x_max]
  x_clamped = clip(x, x_min, x_max)

  # Logistic midpoint
  midpoint = (x_min + x_max) / 2.0

  # Standard logistic formula
  return y_min + (y_max - y_min) / (1.0 + math.e ** (-slope * (x_clamped - midpoint)))


class FrogPilotFollowing:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    self.following_lead = False
    self.slower_lead = False

    # Jerk/follow parameters
    self.acceleration_jerk = 0
    self.base_acceleration_jerk = 0
    self.base_danger_jerk = 0
    self.base_speed_jerk = 0
    self.danger_jerk = 0
    self.speed_jerk = 0
    self.t_follow = 0

    # Additional attributes from old code
    self.safe_obstacle_distance = 0
    self.safe_obstacle_distance_stock = 0
    self.stopped_equivalence_factor = 0

  def update(self, aEgo, controlsState, frogpilotCarState, lead_distance, v_ego, v_lead, frogpilot_toggles):
    """
    Main update:
      - We do NOT scale t_follow with speed anymore—whatever the user sets or
        get_T_FOLLOW returns is used as-is.
      - Jerk is scaled so it goes DOWN at higher speeds.
    """

    # ------------------------------------------------------
    # 1. Handle traffic mode or non-traffic mode (baseline)
    # ------------------------------------------------------
    if frogpilotCarState.trafficModeActive:
      # Use linear interpolation from toggles for base jerk. (No t_follow scaling at speed.)
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
      # Non-traffic mode: baseline from get_jerk_factor() and get_T_FOLLOW()
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
          frogpilot_toggles.relaxed_jerk_speed_decrease,
          frogpilot_toggles.custom_personalities,
          controlsState.personality
        )

      # Use the raw T_FOLLOW, unscaled
      self.t_follow = get_T_FOLLOW(
        frogpilot_toggles.aggressive_follow,
        frogpilot_toggles.standard_follow,
        frogpilot_toggles.relaxed_follow,
        frogpilot_toggles.custom_personalities,
        controlsState.personality
      )

    # ------------------------------------------------------
    # 2. Scale jerk so it is lower at high speeds
    # ------------------------------------------------------
    # Example: At 0 m/s -> ~1.2× jerk; at 40 m/s (~90 mph) -> ~0.8× jerk.
    # Adjust these to your preference (just ensure the first param is bigger
    # than the second to get a downward trend).
    jerk_scale = logistic_interpolate(
      x=v_ego,
      x_min=0.0,             # 0 m/s (stopped)
      x_max=40.0,            # 40 m/s (~90 mph), adjust as needed
      y_min=1.1,             # scale factor at low speed
      y_max=0.7,             # scale factor at high speed
      slope=1.0
    )

    self.acceleration_jerk = self.base_acceleration_jerk * jerk_scale
    self.danger_jerk = self.base_danger_jerk * jerk_scale
    self.speed_jerk = self.base_speed_jerk * jerk_scale

    # ------------------------------------------------------
    # 3. Lead detection and distance
    # ------------------------------------------------------
    self.following_lead = (
      self.frogpilot_planner.tracking_lead
      and (lead_distance < (self.t_follow + 1.0) * v_ego)
    )

    if self.frogpilot_planner.tracking_lead:
      # Ensure these are set for other parts of the code
      self.safe_obstacle_distance = int(get_safe_obstacle_distance(v_ego, self.t_follow))
      self.safe_obstacle_distance_stock = self.safe_obstacle_distance
      self.stopped_equivalence_factor = int(get_stopped_equivalence_factor(v_lead))

      # Fine-tune if faster/slower lead
      self.update_follow_values(lead_distance, v_ego, v_lead, frogpilot_toggles)
    else:
      self.safe_obstacle_distance = 0
      self.safe_obstacle_distance_stock = 0
      self.stopped_equivalence_factor = 0

  def update_follow_values(self, lead_distance, v_ego, v_lead, frogpilot_toggles):
    """
    Fine-tune jerk/follow time if the lead is significantly faster or slower.
    We do NOT scale t_follow with speed here—only in reaction to a lead's behavior.
    """
    # If the lead is faster, we reduce jerk a bit to avoid abrupt speed-ups
    if frogpilot_toggles.human_following and v_lead > v_ego:
      distance_factor = max(lead_distance - (v_ego * self.t_follow), 1)
      standstill_offset = max(STOP_DISTANCE - v_ego, 1)
      acceleration_offset = clip(
        (v_lead - v_ego) * standstill_offset - COMFORT_BRAKE,
        1, distance_factor
      )
      # Damp acceleration and speed jerk
      self.acceleration_jerk /= standstill_offset
      self.speed_jerk /= standstill_offset
      # Also tighten follow time a bit, but only due to the faster lead scenario
      self.t_follow /= acceleration_offset

    # If the lead is slower, or if we’re above cruising speed, we back off a bit
    if ((frogpilot_toggles.conditional_slower_lead or frogpilot_toggles.human_following)
        and v_lead < v_ego > CRUISING_SPEED):
      distance_factor = max(lead_distance - (v_lead * self.t_follow), 1)
      far_lead_offset = max(v_lead - CITY_SPEED_LIMIT, 1)
      braking_offset = clip(
        min(v_ego - v_lead, v_lead) * far_lead_offset - COMFORT_BRAKE,
        1, distance_factor
      )
      if frogpilot_toggles.human_following:
        self.t_follow /= braking_offset
      self.slower_lead = (braking_offset / far_lead_offset) > 1