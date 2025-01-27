# PFEIFER - SLC - Modified by FrogAi for FrogPilot
import json
import math  # Keep math import for smooth_transition

from openpilot.selfdrive.frogpilot.frogpilot_utilities import calculate_distance_to_point
from openpilot.selfdrive.frogpilot.frogpilot_variables import TO_RADIANS, params, params_memory


class SpeedLimitController:
    def __init__(self):
        self.experimental_mode = False
        self.speed_limit_changed = False

        self.desired_speed_limit = 0
        self.map_speed_limit = 0  # Keep map_speed_limit initialization
        self.speed_limit = 0
        self.upcoming_speed_limit = 0

        self.source = "None"

        self.previous_speed_limit = params.get_float("PreviousSpeedLimit")

        # Keep smoothing parameters
        self.accel_limit = 2.0
        self.decel_limit = 3.0
        self.smooth_speed_limit = self.previous_speed_limit

        # Keep offset initialization (though offset is calculated dynamically in get_offset)
        self.offset = 0.0  # Initialize offset


    def update(self, dashboard_speed_limit, enabled, navigation_speed_limit, map_speed_limit_realtime, v_cruise, v_ego, frogpilot_toggles, dt=0.05):  # Keep dt in update signature
        self.process_upcoming_speed_limit(v_ego, frogpilot_toggles)  # Keep process_upcoming_speed_limit name
        self.map_speed_limit = map_speed_limit_realtime  # Keep map_speed_limit update from realtime input
        max_speed_limit = v_cruise if enabled else 0

        self.speed_limit = self.get_speed_limit(dashboard_speed_limit, max_speed_limit, navigation_speed_limit, frogpilot_toggles)

        # Apply offset *before* smoothing and desired speed limit calculation - **CRUCIAL FIX**
        self.offset = self.get_offset(self.speed_limit, frogpilot_toggles) # Calculate offset
        self.speed_limit += self.offset # Apply offset to speed_limit

        self.desired_speed_limit = self.get_desired_speed_limit()
        self.smooth_speed_limit = self.smooth_transition(self.smooth_speed_limit, self.desired_speed_limit, dt)  # Keep smoothing
        self.speed_limit = self.smooth_speed_limit  # Use smoothed speed limit as final speed_limit

        self.experimental_mode = frogpilot_toggles.slc_fallback_experimental_mode and self.speed_limit == 0


    def get_desired_speed_limit(self):  # Keep get_desired_speed_limit
        if self.speed_limit > 1:
            if abs(self.speed_limit - self.previous_speed_limit) > 1:
                params.put_float_nonblocking("PreviousSpeedLimit", self.speed_limit)
                self.previous_speed_limit = self.speed_limit
                self.speed_limit_changed = True
            return self.speed_limit
        else:
            self.speed_limit_changed = False
            return 0


    def process_upcoming_speed_limit(self, v_ego, frogpilot_toggles):  # Keep process_upcoming_speed_limit name
        position = json.loads(params_memory.get("LastGPSPosition") or "{}")
        if not position:
            return  # Do not set map_speed_limit to 0 here

        next_map_speed_limit = json.loads(params_memory.get("NextMapSpeedLimit") or "{}")
        self.upcoming_speed_limit = next_map_speed_limit.get("speedlimit", 0)
        if self.upcoming_speed_limit > 1:
            current_latitude = position.get("latitude")
            current_longitude = position.get("longitude")

            upcoming_latitude = next_map_speed_limit.get("latitude")
            upcoming_longitude = next_map_speed_limit.get("longitude")

            distance_to_upcoming = calculate_distance_to_point(current_latitude * TO_RADIANS, current_longitude * TO_RADIANS, upcoming_latitude * TO_RADIANS, upcoming_longitude * TO_RADIANS)

            if self.previous_speed_limit < self.upcoming_speed_limit:
                max_distance = frogpilot_toggles.map_speed_lookahead_higher * v_ego
            else:
                max_distance = frogpilot_toggles.map_speed_lookahead_lower * v_ego

            if distance_to_upcoming < max_distance:  # Condition from upstream - Use upcoming to set map_speed_limit
                self.map_speed_limit = self.upcoming_speed_limit  # Use upcoming speed limit to update map_speed_limit


    def get_offset(self, speed_limit, frogpilot_toggles):  # Keep get_offset - **CRUCIAL FUNCTION RETAINED**
        """Return offset for the given speed limit range, exactly as in old code."""
        if speed_limit < 1:
            return 0.0
        if speed_limit < 13.5:
            return frogpilot_toggles.speed_limit_offset1
        if speed_limit < 24:
            return frogpilot_toggles.speed_limit_offset2
        if speed_limit < 29:
            return frogpilot_toggles.speed_limit_offset3
        return frogpilot_toggles.speed_limit_offset4


    def get_speed_limit(self, dashboard_speed_limit, max_speed_limit, navigation_speed_limit, frogpilot_toggles):  # Keep get_speed_limit
        limits = {
            "Dashboard": dashboard_speed_limit,
            "Map Data": self.map_speed_limit,  # Use self.map_speed_limit
            "Navigation": navigation_speed_limit
        }
        filtered_limits = {source: float(limit) for source, limit in limits.items() if limit > 1}

        if filtered_limits:
            if frogpilot_toggles.speed_limit_priority_highest:
                self.source = max(filtered_limits, key=filtered_limits.get)
                return filtered_limits[self.source]

            if frogpilot_toggles.speed_limit_priority_lowest:
                self.source = min(filtered_limits, key=filtered_limits.get)
                return filtered_limits[self.source]

            for priority in [
                frogpilot_toggles.speed_limit_priority1,
                frogpilot_toggles.speed_limit_priority2,
                frogpilot_toggles.speed_limit_priority3
            ]:
                if priority is not None and priority in filtered_limits:
                    self.source = priority
                    return filtered_limits[priority]

        self.source = "None"

        if frogpilot_toggles.slc_fallback_previous_speed_limit:
            return self.previous_speed_limit

        if frogpilot_toggles.slc_fallback_set_speed:
            return max_speed_limit

        return 0


    def smooth_transition(self, current_smooth, target_speed, dt):  # Keep smooth_transition - **CRUCIAL FUNCTION RETAINED**
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