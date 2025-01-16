import json
import math

from openpilot.selfdrive.frogpilot.frogpilot_utilities import calculate_distance_to_point
from openpilot.selfdrive.frogpilot.frogpilot_variables import TO_RADIANS, params, params_memory

class SpeedLimitController:
  def __init__(self):
    """Initialize the SpeedLimitController with necessary state variables."""

    # Whether or not we should fallback to "experimental mode"
    # (e.g. use openpilot's experimental features if no valid limit)
    self.experimental_mode = False

    # The raw (base) speed limit from dash/map/nav, before offset
    self.speed_limit = 0

    # The user-set offset above (or below) the raw limit
    self.offset = 0

    # The "instantaneous" or "desired" final speed = raw limit + offset
    self.desired_speed_limit = 0

    # A smoothed version of the desired speed limit (to avoid abrupt transitions)
    self.smooth_speed_limit = 0.0

    # Map-based speed limits
    self.map_speed_limit = 0
    self.upcoming_speed_limit = 0

    # The most recent source of the speed limit among: Dashboard / Map / Navigation / None
    self.source = "None"

    # Load previously stored speed limit from persistent params
    self.previous_speed_limit = params.get_float("PreviousSpeedLimit")
    if (math.isnan(self.previous_speed_limit) or
        self.previous_speed_limit < 0 or
        self.previous_speed_limit > 150):
      # Clamp to 0 if invalid or out of range
      self.previous_speed_limit = 0

    # Define maximum acceleration/deceleration rates for speed changes (in MPH/second)
    # so transitions feel more human-like
    self.accel_limit = 2.0   # e.g. accelerate up to +2 MPH per second
    self.decel_limit = 3.0   # e.g. decelerate up to -3 MPH per second

    # Initialize smoothed limit to the last known limit (so we don't start from 0 unexpectedly)
    self.smooth_speed_limit = self.previous_speed_limit

  def update(self,
             dashboard_speed_limit,
             enabled,
             navigation_speed_limit,
             v_cruise,
             v_ego,
             frogpilot_toggles,
             dt=0.05):
    """
    Main update function, typically called periodically (e.g. 20Hz).
    
    :param dashboard_speed_limit: The speed limit detected by dash or OCR-based sign reading (float)
    :param enabled: Whether openpilot is currently active/engaged (bool)
    :param navigation_speed_limit: The speed limit from route navigation (float)
    :param v_cruise: The user-set cruise speed (float, mph)
    :param v_ego: Current vehicle speed (float, mph)
    :param frogpilot_toggles: A structure or class that holds various user-tunable settings
    :param dt: Time delta for each update call (seconds), e.g. 0.05 if at 20Hz
    """

    # 1) Possibly update speed limit based on map data
    self.update_map_speed_limit(v_ego, frogpilot_toggles)

    # 2) If system is enabled, we can fall back to v_cruise; otherwise 0
    max_speed_limit = v_cruise if enabled else 0

    # 3) Retrieve the raw/base speed limit from your chosen priority source
    base_limit = self.get_speed_limit(
      dashboard_speed_limit,
      max_speed_limit,
      navigation_speed_limit,
      frogpilot_toggles
    )
    self.speed_limit = base_limit  # This remains separate from the offset

    # 4) Determine offset for this base speed limit
    self.offset = self.get_offset(self.speed_limit, frogpilot_toggles)

    # 5) Compute the instantaneous "desired" speed limit (base + offset)
    #    while preserving the original threshold logic to store "previous" limit
    self.desired_speed_limit = self.get_desired_speed_limit()

    # 6) Smoothly transition to the desired speed limit over time,
    #    rather than jumping immediately
    self.smooth_speed_limit = self.smooth_transition(
      current_smooth=self.smooth_speed_limit,
      target_speed=self.desired_speed_limit,
      dt=dt
    )

    # 7) Possibly enable experimental mode if no valid speed limit found
    self.experimental_mode = (
      frogpilot_toggles.slc_fallback_experimental_mode and
      self.speed_limit == 0
    )

  def get_desired_speed_limit(self):
    """
    Return the "instant" target speed limit = raw (base) speed limit + offset.
    Also updates `self.previous_speed_limit` in persistent storage if the new
    final speed changes by more than 1 mph from the previously stored value.
    """
    if self.speed_limit > 1:
      final_speed = self.speed_limit + self.offset
      # If the final speed differs enough from our stored previous speed, update it
      if abs(final_speed - self.previous_speed_limit) > 1:
        params.put_float_nonblocking("PreviousSpeedLimit", final_speed)
        self.previous_speed_limit = final_speed
      return final_speed
    else:
      # If there's no valid base limit, return 0 and zero out the offset
      self.offset = 0
      return 0

  def smooth_transition(self, current_smooth, target_speed, dt):
    """
    Gently move our smoothed speed limit toward `target_speed` to avoid abrupt changes.
    This uses simple linear “acceleration” or “deceleration” limits, measured in MPH/sec.
    """
    # If the target speed is invalid, snap to 0
    if target_speed <= 0:
      return 0.0

    # How far away we are from the target
    diff = target_speed - current_smooth

    # If we're basically there, do nothing
    if abs(diff) < 0.001:
      return current_smooth

    # Depending on whether we're accelerating or decelerating, pick a rate
    rate = self.accel_limit if diff > 0 else self.decel_limit

    # The maximum we can change in this time step
    step = rate * dt

    if abs(diff) <= step:
      # We can reach the target in this single step
      return target_speed
    else:
      # Move a small step toward the target
      return current_smooth + math.copysign(step, diff)

  def update_map_speed_limit(self, v_ego, frogpilot_toggles):
    """
    If there's an upcoming speed limit in the param store (e.g. from map data),
    switch to it if we're within the specified lookahead distance. Otherwise, keep current map speed limit.
    """
    self.map_speed_limit = params_memory.get_float("MapSpeedLimit")

    next_map_speed_limit = json.loads(params_memory.get("NextMapSpeedLimit", "{}"))
    self.upcoming_speed_limit = next_map_speed_limit.get("speedlimit", 0)

    if self.upcoming_speed_limit > 1:
      position = json.loads(params_memory.get("LastGPSPosition", "{}"))
      latitude = position.get("latitude", 0)
      longitude = position.get("longitude", 0)

      next_lat = next_map_speed_limit.get("latitude", 0)
      next_lon = next_map_speed_limit.get("longitude", 0)

      distance = calculate_distance_to_point(
        latitude * TO_RADIANS,
        longitude * TO_RADIANS,
        next_lat * TO_RADIANS,
        next_lon * TO_RADIANS
      )

      # Check distance against different lookahead multipliers for inc vs. dec
      if self.previous_speed_limit < self.upcoming_speed_limit:
        max_distance = frogpilot_toggles.map_speed_lookahead_higher * v_ego
      else:
        max_distance = frogpilot_toggles.map_speed_lookahead_lower * v_ego

      # If within lookahead distance, use the upcoming speed limit
      if distance < max_distance:
        self.map_speed_limit = self.upcoming_speed_limit
    # Otherwise, leave self.map_speed_limit alone (it might be 0 if no map data)

  def get_offset(self, speed_limit, frogpilot_toggles):
    """
    Return the offset (above or below the base limit) depending on ranges or user config.
    We do NOT ever subtract it from the base limit — we are always adding the offset
    to the raw limit to get the final speed.
    """
    if speed_limit < 1:
      # Invalid or zero base limit
      return 0
    elif speed_limit < 13.5:
      return frogpilot_toggles.speed_limit_offset1
    elif speed_limit < 24:
      return frogpilot_toggles.speed_limit_offset2
    elif speed_limit < 29:
      return frogpilot_toggles.speed_limit_offset3
    else:
      return frogpilot_toggles.speed_limit_offset4

  def get_speed_limit(self,
                      dashboard_speed_limit,
                      max_speed_limit,
                      navigation_speed_limit,
                      frogpilot_toggles):
    """
    Determine the raw/base speed limit from the chosen priorities:
      1) "Dashboard" (OCR/dash sign detection)
      2) "Map Data"
      3) "Navigation"
    
    Then choose according to the user’s priority selection (highest/lowest/priority1->2->3).
    If none is valid, fallback to "previous speed limit" or "set cruise speed" if toggles allow.
    """
    # Collect potential speed limits that are > 1 mph
    limits = {
      "Dashboard": dashboard_speed_limit,
      "Map Data": self.map_speed_limit,
      "Navigation": navigation_speed_limit
    }
    filtered_limits = {src: float(lim) for src, lim in limits.items() if lim > 1}

    if filtered_limits:
      # If user wants the highest limit among available
      if frogpilot_toggles.speed_limit_priority_highest:
        self.source = max(filtered_limits, key=filtered_limits.get)
        return filtered_limits[self.source]

      # If user wants the lowest limit among available
      if frogpilot_toggles.speed_limit_priority_lowest:
        self.source = min(filtered_limits, key=filtered_limits.get)
        return filtered_limits[self.source]

      # Otherwise check priority1 -> priority2 -> priority3 in order
      for priority in [
        frogpilot_toggles.speed_limit_priority1,
        frogpilot_toggles.speed_limit_priority2,
        frogpilot_toggles.speed_limit_priority3
      ]:
        if priority is not None and priority in filtered_limits:
          self.source = priority
          return filtered_limits[priority]

    # If no valid limit found or all are <= 1 mph, fallback
    self.source = "None"

    # 1) If toggled on, fallback to previously stored speed limit
    if frogpilot_toggles.slc_fallback_previous_speed_limit:
      if 1 < self.previous_speed_limit < 150:
        return self.previous_speed_limit
      else:
        return 0

    # 2) If toggled on, fallback to the user’s current set cruise speed
    if frogpilot_toggles.slc_fallback_set_speed:
      return max_speed_limit if max_speed_limit > 1 else 0

    # Otherwise, no valid speed limit
    return 0