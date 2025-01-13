# PFEIFER - SLC - Modified by FrogAi for FrogPilot
import json

from openpilot.selfdrive.frogpilot.frogpilot_utilities import calculate_distance_to_point
from openpilot.selfdrive.frogpilot.frogpilot_variables import TO_RADIANS, params, params_memory

class SpeedLimitController:
  def __init__(self):
    self.experimental_mode = False

    self.desired_speed_limit = 0
    self.map_speed_limit = 0
    self.speed_limit = 0
    self.upcoming_speed_limit = 0

    self.source = "None"

    self.previous_speed_limit = params.get_float("PreviousSpeedLimit")

  def update(self, dashboard_speed_limit, enabled, navigation_speed_limit, v_cruise, v_ego, frogpilot_toggles):
    # Update possible map-based speed limit
    self.update_map_speed_limit(v_ego, frogpilot_toggles)

    # If system is enabled, use v_cruise as fallback set speed; otherwise 0
    max_speed_limit = v_cruise if enabled else 0

    # Get the raw base speed limit from whichever source is set to priority
    self.speed_limit = self.get_speed_limit(
      dashboard_speed_limit,
      max_speed_limit,
      navigation_speed_limit,
      frogpilot_toggles
    )

    # Get the appropriate offset for the detected base limit
    self.offset = self.get_offset(self.speed_limit, frogpilot_toggles)

    # Compute the final speed limit (base + offset)
    self.desired_speed_limit = self.get_desired_speed_limit()

    # If no speed limit found (i.e., zero), optionally enable experimental mode
    self.experimental_mode = (
      frogpilot_toggles.slc_fallback_experimental_mode and
      self.speed_limit == 0
    )

  def get_desired_speed_limit(self):
    """
    Returns the final target speed limit = raw (base) speed limit + offset.
    Leaves self.speed_limit unchanged so that the HUD can display the correct base.
    """
    if self.speed_limit > 1:
      # If the base speed limit has changed significantly from stored, update it
      if abs(self.speed_limit - self.previous_speed_limit) > 1:
        params.put_float_nonblocking("PreviousSpeedLimit", self.speed_limit)
        self.previous_speed_limit = self.speed_limit
      return self.speed_limit + self.offset
    else:
      return 0

  def update_map_speed_limit(self, v_ego, frogpilot_toggles):
    self.map_speed_limit = params_memory.get_float("MapSpeedLimit")

    next_map_speed_limit = json.loads(params_memory.get("NextMapSpeedLimit", "{}"))
    self.upcoming_speed_limit = next_map_speed_limit.get("speedlimit", 0)

    # If there's an upcoming speed limit, we may want to switch to it if close enough
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

      # Depending on whether it's higher or lower, pick a different lookahead distance
      if self.previous_speed_limit < self.upcoming_speed_limit:
        max_distance = frogpilot_toggles.map_speed_lookahead_higher * v_ego
      else:
        max_distance = frogpilot_toggles.map_speed_lookahead_lower * v_ego

      if distance < max_distance:
        self.map_speed_limit = self.upcoming_speed_limit

  def get_offset(self, speed_limit, frogpilot_toggles):
    """
    Returns an offset based on the current speed limit range,
    e.g., 10 mph over if < 55 mph, 5 mph over if 55+, etc.
    """
    if speed_limit < 13.5:
      return frogpilot_toggles.speed_limit_offset1
    elif speed_limit < 24:
      return frogpilot_toggles.speed_limit_offset2
    elif speed_limit < 29:
      return frogpilot_toggles.speed_limit_offset3
    else:
      return frogpilot_toggles.speed_limit_offset4

  def get_speed_limit(self, dashboard_speed_limit, max_speed_limit, navigation_speed_limit, frogpilot_toggles):
    # Gather all potential speed limits above 1 mph
    limits = {
      "Dashboard": dashboard_speed_limit,
      "Map Data": self.map_speed_limit,
      "Navigation": navigation_speed_limit
    }
    filtered_limits = {source: float(limit) for source, limit in limits.items() if limit > 1}

    if filtered_limits:
      # If "highest" priority is active, pick the largest limit
      if frogpilot_toggles.speed_limit_priority_highest:
        self.source = max(filtered_limits, key=filtered_limits.get)
        return filtered_limits[self.source]

      # If "lowest" priority is active, pick the smallest limit
      if frogpilot_toggles.speed_limit_priority_lowest:
        self.source = min(filtered_limits, key=filtered_limits.get)
        return filtered_limits[self.source]

      # Otherwise, follow the priority1 -> priority2 -> priority3 chain
      for priority in [
        frogpilot_toggles.speed_limit_priority1,
        frogpilot_toggles.speed_limit_priority2,
        frogpilot_toggles.speed_limit_priority3
      ]:
        if priority is not None and priority in filtered_limits:
          self.source = priority
          return filtered_limits[priority]

    # If we didn't find a valid speed limit from above, fallback logic
    self.source = "None"

    # Fallback to previous speed limit if enabled
    if frogpilot_toggles.slc_fallback_previous_speed_limit:
      return self.previous_speed_limit

    # Fallback to the set cruise speed (v_cruise) if enabled
    if frogpilot_toggles.slc_fallback_set_speed:
      self.offset = 0
      return max_speed_limit

    # Otherwise, no valid speed limit
    return 0