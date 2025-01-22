import json
import math

from openpilot.selfdrive.frogpilot.frogpilot_utilities import calculate_distance_to_point
from openpilot.selfdrive.frogpilot.frogpilot_variables import TO_RADIANS, params, params_memory


class SpeedLimitController:
  """
  A controller that determines a final speed limit based on multiple sources (dashboard, map, navigation),
  user preferences (offsets, fallback logic), and a smoothing function that avoids abrupt changes.
  """

  def __init__(self):
    """
    Initialize internal state, including last speed limit from persistent params.
    """
    # Experimental mode fallback (enabled if no valid limit is found)
    self.experimental_mode = False

    # Currently selected "raw" speed limit from the chosen source
    self.speed_limit = 0.0

    # The user-defined offset above/below the raw limit
    self.offset = 0.0

    # The instantaneous (raw + offset) desired speed limit
    self.desired_speed_limit = 0.0

    # Smoothed version of desired_speed_limit to avoid abrupt jumps
    self.smooth_speed_limit = 0.0

    # Map-based speed limits (current and upcoming)
    self.map_speed_limit = 0.0
    self.upcoming_speed_limit = 0.0

    # Most recent source of the speed limit
    self.source = "None"

    # Load previously stored speed limit from persistent params
    self.previous_speed_limit = params.get_float("PreviousSpeedLimit")
    # Clamp to 0 if invalid/NaN/out of range
    if not (1 <= self.previous_speed_limit <= 150):
      self.previous_speed_limit = 0.0

    # Accel/decel limits for smoothing (MPH/second)
    self.accel_limit = 2.0
    self.decel_limit = 3.0

    # Initialize smoothed speed limit to the stored value
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
    Main update function. Typically called at ~20Hz.

    :param dashboard_speed_limit:  Speed limit from dash/vision sign detection.
    :param enabled:                Boolean indicating if openpilot is engaged.
    :param navigation_speed_limit: Speed limit from route navigation.
    :param v_cruise:               User-set cruise speed (mph).
    :param v_ego:                  Current vehicle speed (mph).
    :param frogpilot_toggles:      Structure containing user overrides and toggles.
    :param dt:                     Time step in seconds since last update (0.05 at 20Hz).
    """
    # 1) Update map-based speed limits from param memory (just like old code)
    self.update_map_speed_limit(v_ego, frogpilot_toggles)

    # 2) If engaged, fallback to v_cruise if no valid limit; otherwise 0
    max_speed_limit = v_cruise if enabled else 0.0

    # 3) Pick a raw speed limit from dash/map/nav, respecting user priority
    self.speed_limit = self.get_speed_limit(
      dashboard_speed_limit,
      max_speed_limit,
      navigation_speed_limit,
      frogpilot_toggles
    )

    # 4) Determine the offset (above the raw limit) based on the chosen limit's range
    self.offset = self.get_offset(self.speed_limit, frogpilot_toggles)

    # 5) Compute the "instantaneous" final speed limit = raw limit + offset
    self.desired_speed_limit = self.compute_desired_speed_limit()

    # 6) Smoothly approach that final speed limit
    self.smooth_speed_limit = self.smooth_transition(
      current_smooth=self.smooth_speed_limit,
      target_speed=self.desired_speed_limit,
      dt=dt
    )

    # 7) Possibly enable experimental mode if no valid speed limit is found
    self.experimental_mode = (frogpilot_toggles.slc_fallback_experimental_mode and
                              self.speed_limit == 0)

  def update_map_speed_limit(self, v_ego, frogpilot_toggles):
    """
    Exactly the same map ingestion logic as the old script: read from param memory,
    check distance to next speed limit, adopt it if we're within the lookahead threshold.
    """
    # Current map limit from param memory (convert from m/s to mph)
    map_speed_limit_ms = params_memory.get_float("MapSpeedLimit")
    self.map_speed_limit = map_speed_limit_ms * 2.23694 if map_speed_limit_ms > 0 else 0

    # Next upcoming speed limit info from param memory
    next_map_speed_limit = json.loads(params_memory.get("NextMapSpeedLimit", "{}"))
    self.upcoming_speed_limit = next_map_speed_limit.get("speedlimit", 0)

    # If there's a valid upcoming speed limit, see if we should switch to it
    if self.upcoming_speed_limit > 1:
      position = json.loads(params_memory.get("LastGPSPosition", "{}"))
      latitude = position.get("latitude", 0.0)
      longitude = position.get("longitude", 0.0)

      next_lat = next_map_speed_limit.get("latitude", 0.0)
      next_lon = next_map_speed_limit.get("longitude", 0.0)

      distance = calculate_distance_to_point(
        latitude * TO_RADIANS,
        longitude * TO_RADIANS,
        next_lat * TO_RADIANS,
        next_lon * TO_RADIANS
      )

      # If upcoming is higher or lower than our previous speed limit, use different lookahead
      if self.previous_speed_limit < self.upcoming_speed_limit:
        max_distance = frogpilot_toggles.map_speed_lookahead_higher * v_ego
      else:
        max_distance = frogpilot_toggles.map_speed_lookahead_lower * v_ego

      # If we are within that distance, adopt the upcoming limit
      if distance < max_distance:
        self.map_speed_limit = self.upcoming_speed_limit

  def get_speed_limit(self,
                      dashboard_speed_limit,
                      max_speed_limit,
                      navigation_speed_limit,
                      frogpilot_toggles):
    """
    Gather speed limits from dash, map, and navigation, then pick one according
    to user-defined priorities. Fallback to previous speed or set speed if toggles allow.

    Exactly the same as the old code's logic.
    """
    # Collect potential limits
    limits = {
      "Dashboard": dashboard_speed_limit,
      "Map Data": self.map_speed_limit,
      "Navigation": navigation_speed_limit
    }
    # Only keep those > 1 mph
    filtered_limits = {
      source: float(lim) for source, lim in limits.items() if lim > 1
    }

    # If we have at least one valid speed limit
    if filtered_limits:
      # If user wants highest among the available
      if frogpilot_toggles.speed_limit_priority_highest:
        self.source = max(filtered_limits, key=filtered_limits.get)
        return filtered_limits[self.source]

      # If user wants lowest among the available
      if frogpilot_toggles.speed_limit_priority_lowest:
        self.source = min(filtered_limits, key=filtered_limits.get)
        return filtered_limits[self.source]

      # Otherwise follow the user's priority list
      for priority in [
        frogpilot_toggles.speed_limit_priority1,
        frogpilot_toggles.speed_limit_priority2,
        frogpilot_toggles.speed_limit_priority3
      ]:
        if priority is not None and priority in filtered_limits:
          self.source = priority
          return filtered_limits[priority]

    # If no valid speed limit found or all are <= 1 mph
    self.source = "None"

    # Fallback to previously stored speed limit if enabled
    if frogpilot_toggles.slc_fallback_previous_speed_limit:
      if 1 < self.previous_speed_limit < 150:
        return self.previous_speed_limit
      return 0.0

    # Fallback to the set cruise speed if enabled
    if frogpilot_toggles.slc_fallback_set_speed:
      return max_speed_limit if max_speed_limit > 1 else 0.0

    # Otherwise, no valid speed limit => 0
    return 0.0

  def compute_desired_speed_limit(self):
    """
    Return the raw limit + offset if valid, else 0.
    Also updates persistent params if the new speed changes > 1 mph from the stored limit.
    """
    if self.speed_limit <= 1:
      return 0.0

    new_final_speed = self.speed_limit + self.offset

    if abs(new_final_speed - self.previous_speed_limit) > 1:
      params.put_float_nonblocking("PreviousSpeedLimit", new_final_speed)
      self.previous_speed_limit = new_final_speed

    return new_final_speed

  def get_offset(self, speed_limit, frogpilot_toggles):
    """
    Return offset for the given speed limit range, exactly as in old code.
    """
    if speed_limit < 1:
      return 0.0
    if speed_limit < 13.5:
      return frogpilot_toggles.speed_limit_offset1
    if speed_limit < 24:
      return frogpilot_toggles.speed_limit_offset2
    if speed_limit < 29:
      return frogpilot_toggles.speed_limit_offset3
    return frogpilot_toggles.speed_limit_offset4

  def smooth_transition(self, current_smooth, target_speed, dt):
    """
    Gently move current_smooth -> target_speed using linear accel/decel limits.
    If target_speed <= 0, snap to zero. If diff < step, jump immediately.
    Otherwise move by step in the appropriate direction.
    """
    # If invalid target, clamp to 0
    if target_speed <= 0:
      return 0.0

    diff = target_speed - current_smooth
    if abs(diff) < 0.001:
      # Already basically at the target
      return current_smooth

    # Decide whether we're accelerating or decelerating
    rate = self.accel_limit if diff > 0 else self.decel_limit

    # The maximum possible change in this time step
    step = rate * dt
    if abs(diff) <= step:
      return target_speed
    else:
      return current_smooth + math.copysign(step, diff)
