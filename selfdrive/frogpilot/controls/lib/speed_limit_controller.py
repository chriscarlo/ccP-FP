# PFEIFER - SLC - Modified by FrogAi for FrogPilot
import json
import math

from openpilot.selfdrive.frogpilot.frogpilot_utilities import calculate_distance_to_point
from openpilot.selfdrive.frogpilot.frogpilot_variables import TO_RADIANS, params, params_memory

class SpeedLimitController:
  def __init__(self):
    self.experimental_mode = False

    self.desired_speed_limit = 0
    self.map_speed_limit = 0
    self.speed_limit = 0            # "Raw" or "base" limit (from dash/map/etc.)
    self.upcoming_speed_limit = 0
    self.offset = 0                 # User-configured offset above the base limit

    self.source = "None"

    # Pull previous speed limit from params
    self.previous_speed_limit = params.get_float("PreviousSpeedLimit")

    # 1) Clamp it in case it’s invalid or out of range
    if (math.isnan(self.previous_speed_limit) or
        self.previous_speed_limit < 0 or
        self.previous_speed_limit > 150):
      self.previous_speed_limit = 0

  def update(self, dashboard_speed_limit, enabled, navigation_speed_limit,
             v_cruise, v_ego, frogpilot_toggles):
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
    self.speed_limit = base_limit

    # 4) Determine offset for this base speed limit
    self.offset = self.get_offset(self.speed_limit, frogpilot_toggles)

    # 5) Compute the final "desired" speed (base + offset) while retaining
    #    the original "smooth transition" threshold logic
    self.desired_speed_limit = self.get_desired_speed_limit()

    # 6) Possibly enable experimental mode if no valid speed limit found
    self.experimental_mode = (
      frogpilot_toggles.slc_fallback_experimental_mode and
      self.speed_limit == 0
    )

  def get_desired_speed_limit(self):
    """
    Return the final target speed limit = raw (base) speed limit + offset,
    while preserving the original threshold check for storing previous limit.
    """
    if self.speed_limit > 1:
      final_speed = self.speed_limit + self.offset

      # Original threshold logic: only update stored "previous" limit
      # if the new final speed differs by more than 1 mph
      if abs(final_speed - self.previous_speed_limit) > 1:
        params.put_float_nonblocking("PreviousSpeedLimit", final_speed)
        self.previous_speed_limit = final_speed

      return final_speed
    else:
      # If there's no valid base limit, it's safer to return 0,
      # and also zero out the offset
      self.offset = 0
      return 0

  def update_map_speed_limit(self, v_ego, frogpilot_toggles):
    """
    Check if there's an upcoming speed limit from map data and possibly
    switch to it if we're within the specified lookahead distance.
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

      # Different lookahead distances for speed limit increases vs. decreases
      if self.previous_speed_limit < self.upcoming_speed_limit:
        max_distance = frogpilot_toggles.map_speed_lookahead_higher * v_ego
      else:
        max_distance = frogpilot_toggles.map_speed_lookahead_lower * v_ego

      if distance < max_distance:
        self.map_speed_limit = self.upcoming_speed_limit
    else:
      # If there's no upcoming map-based limit, just keep the stored map_speed_limit
      # or it might be 0 if none is available
      pass

  def get_offset(self, speed_limit, frogpilot_toggles):
    """
    Return an offset (above the base limit) depending on the range that
    'speed_limit' falls into. For example:
      - If speed < 13.5 mph, use offset1
      - If speed < 24 mph, use offset2
      - If speed < 29 mph, use offset3
      - Otherwise use offset4
    """
    if speed_limit < 1:
      # If it's 0 or invalid, no offset
      return 0
    elif speed_limit < 13.5:
      return frogpilot_toggles.speed_limit_offset1
    elif speed_limit < 24:
      return frogpilot_toggles.speed_limit_offset2
    elif speed_limit < 29:
      return frogpilot_toggles.speed_limit_offset3
    else:
      return frogpilot_toggles.speed_limit_offset4

  def get_speed_limit(self, dashboard_speed_limit, max_speed_limit,
                      navigation_speed_limit, frogpilot_toggles):
    """
    Determine the raw/base speed limit by looking at:
      - "Dashboard": OCR-based or dash-based sign detection
      - "Map Data": data from the map
      - "Navigation": from route info
    Then apply whichever "priority" setting the user has selected,
    or fallback as needed.
    """
    # Gather all potential speed limits that are > 1 mph
    limits = {
      "Dashboard": dashboard_speed_limit,
      "Map Data": self.map_speed_limit,
      "Navigation": navigation_speed_limit
    }
    filtered_limits = {source: float(limit) for source, limit in limits.items() if limit > 1}

    if filtered_limits:
      # If "highest" priority is chosen, pick the largest available limit
      if frogpilot_toggles.speed_limit_priority_highest:
        self.source = max(filtered_limits, key=filtered_limits.get)
        return filtered_limits[self.source]

      # If "lowest" priority is chosen, pick the smallest available limit
      if frogpilot_toggles.speed_limit_priority_lowest:
        self.source = min(filtered_limits, key=filtered_limits.get)
        return filtered_limits[self.source]

      # Otherwise, check priority1 -> priority2 -> priority3 in order
      for priority in [
        frogpilot_toggles.speed_limit_priority1,
        frogpilot_toggles.speed_limit_priority2,
        frogpilot_toggles.speed_limit_priority3
      ]:
        if priority is not None and priority in filtered_limits:
          self.source = priority
          return filtered_limits[priority]

    # If we end up here, no valid limit found or all were <= 1
    self.source = "None"

    # Fallback: use stored "previous" speed limit if toggled on and valid
    if frogpilot_toggles.slc_fallback_previous_speed_limit:
      # Ensure that it's within a sane range
      if 1 < self.previous_speed_limit < 150:
        return self.previous_speed_limit
      else:
        return 0

    # Fallback: use set cruise speed if toggled on
    if frogpilot_toggles.slc_fallback_set_speed:
      # We’ll zero offset in get_desired_speed_limit() if this is 0
      return max_speed_limit if max_speed_limit > 1 else 0

    # Otherwise, no valid speed limit
    return 0