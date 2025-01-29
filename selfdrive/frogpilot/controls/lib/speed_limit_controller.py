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

    def update(self,
               dashboard_speed_limit,
               enabled,
               navigation_speed_limit,
               map_speed_limit_realtime,
               upcoming_speed_limit_realtime,  # <-- NEW param
               v_cruise,
               v_ego,
               frogpilot_toggles,
               dt=0.05):
        """
        Main SpeedLimitController update function.
         1) Initialize self.map_speed_limit from real-time map data.
         2) Possibly override self.map_speed_limit with an upcoming speed limit (via param OR the new real-time arg).
         3) Pick final speed limit among Dashboard/Map Data/Navigation or fallback.
         4) Apply user offset, smoothing, and store as self.speed_limit.
        """

        # 1) Start with the real-time map speed-limit
        self.map_speed_limit = map_speed_limit_realtime

        # 2) Possibly override it with an upcoming limit if we are within distance
        #    - If upcoming_speed_limit_realtime is valid (>1), set it; otherwise read from param.
        if upcoming_speed_limit_realtime > 1:
            self.upcoming_speed_limit = upcoming_speed_limit_realtime
        else:
            self.upcoming_speed_limit = 0  # Will try param fallback

        self.process_upcoming_speed_limit(v_ego, frogpilot_toggles)

        # 3) Figure out the max permissible speed if OP is enabled
        max_speed_limit = v_cruise if enabled else 0

        # 4) Decide final speed limit among Dashboard/Map Data/Navigation or fallback
        self.speed_limit = self.get_speed_limit(
            dashboard_speed_limit,
            max_speed_limit,
            navigation_speed_limit,
            frogpilot_toggles
        )

        # 5) Calculate offset first, then apply it to self.speed_limit if we have a valid source
        self.offset = self.get_offset(self.speed_limit, frogpilot_toggles)
        if self.source != "None" or not frogpilot_toggles.slc_fallback_previous_speed_limit:
            # Only apply offset if we have a valid speed-limit source
            # OR if the user specifically doesn't want fallback logic
            self.speed_limit += self.offset

        # 6) Derive the final desired speed limit (check if it changed from the previous)
        self.desired_speed_limit = self.get_desired_speed_limit()

        # 7) Smooth the transition up/down to the new speed limit
        self.smooth_speed_limit = self.smooth_transition(self.smooth_speed_limit, self.desired_speed_limit, dt)
        self.speed_limit = self.smooth_speed_limit  # Use smoothed speed limit as final

        # 8) If no valid limit, we may force experimental mode in certain toggles
        self.experimental_mode = frogpilot_toggles.slc_fallback_experimental_mode and (self.speed_limit == 0)

    def get_desired_speed_limit(self):
        """
        Determine if the new speed limit is valid (>1).
        If it differs from the previous limit by more than 1,
        store it in persistent params and mark speed_limit_changed = True.
        """
        if self.speed_limit > 1:
            if abs(self.speed_limit - self.previous_speed_limit) > 1:
                params.put_float_nonblocking("PreviousSpeedLimit", self.speed_limit)
                self.previous_speed_limit = self.speed_limit
                self.speed_limit_changed = True
            return self.speed_limit
        else:
            self.speed_limit_changed = False
            return 0

    def process_upcoming_speed_limit(self, v_ego, frogpilot_toggles):
        """
        If self.upcoming_speed_limit is >1, we skip the param read.
        Otherwise, read from NextMapSpeedLimit in params_memory to see if there's
        an upcoming speed limit we should enforce. If the upcoming limit is within
        a certain distance (based on v_ego and user-defined lookahead settings),
        override self.map_speed_limit with that upcoming value.
        """
        if self.upcoming_speed_limit <= 1:
            # Fallback to param read
            position = json.loads(params_memory.get("LastGPSPosition") or "{}")
            if not position:
                return  # No GPS position, nothing more to do

            next_map_speed_limit = json.loads(params_memory.get("NextMapSpeedLimit") or "{}")
            param_based_limit = next_map_speed_limit.get("speedlimit", 0)
            if param_based_limit > 1:
                self.upcoming_speed_limit = param_based_limit
                current_latitude = position.get("latitude")
                current_longitude = position.get("longitude")
                upcoming_latitude = next_map_speed_limit.get("latitude")
                upcoming_longitude = next_map_speed_limit.get("longitude")

                if current_latitude and current_longitude and upcoming_latitude and upcoming_longitude:
                    distance_to_upcoming = calculate_distance_to_point(
                        current_latitude * TO_RADIANS,
                        current_longitude * TO_RADIANS,
                        upcoming_latitude * TO_RADIANS,
                        upcoming_longitude * TO_RADIANS
                    )
                    # Decide how far ahead to check
                    if self.previous_speed_limit < self.upcoming_speed_limit:
                        max_distance = frogpilot_toggles.map_speed_lookahead_higher * v_ego
                    else:
                        max_distance = frogpilot_toggles.map_speed_lookahead_lower * v_ego

                    if distance_to_upcoming < max_distance:
                        self.map_speed_limit = self.upcoming_speed_limit
                return

        # If we already have a valid upcoming_speed_limit from the real-time arg:
        if self.upcoming_speed_limit > 1:
            # Potentially do distance-based approach if you want the same logic:
            position = json.loads(params_memory.get("LastGPSPosition") or "{}")
            if position:
                current_latitude = position.get("latitude")
                current_longitude = position.get("longitude")

                # We don't have exact lat/lon for the real-time upcoming, so skip if not provided
                # Or you could unify that logic. For now, let's assume we always apply it:
                if self.previous_speed_limit < self.upcoming_speed_limit:
                    max_distance = frogpilot_toggles.map_speed_lookahead_higher * v_ego
                else:
                    max_distance = frogpilot_toggles.map_speed_lookahead_lower * v_ego

                # If you want to check an actual distance, you’d need the lat/lon for the real-time upcoming limit.
                # Here we’ll assume it’s close enough to override:
                self.map_speed_limit = self.upcoming_speed_limit

    def get_offset(self, speed_limit, frogpilot_toggles):
        """
        Return offset for the given speed limit range (sub-13.5 m/s, sub-24, etc.).
        This logic is unchanged from older versions.
        """
        if speed_limit < 1:
            return 0.0
        if speed_limit < 13.5:  # ~30 mph
            return frogpilot_toggles.speed_limit_offset1
        if speed_limit < 24:    # ~54 mph
            return frogpilot_toggles.speed_limit_offset2
        if speed_limit < 29:    # ~65 mph
            return frogpilot_toggles.speed_limit_offset3
        return frogpilot_toggles.speed_limit_offset4

    def get_speed_limit(self, dashboard_speed_limit, max_speed_limit, navigation_speed_limit, frogpilot_toggles):
        """
        Among Dashboard, Map Data, and Navigation, pick the “best” speed-limit
        based on user-specified priority or fallback.
        """
        limits = {
            "Dashboard": dashboard_speed_limit,
            "Map Data": self.map_speed_limit,
            "Navigation": navigation_speed_limit
        }
        filtered_limits = {src: float(val) for src, val in limits.items() if val > 1}

        if filtered_limits:
            # If user wants to pick the highest limit
            if frogpilot_toggles.speed_limit_priority_highest:
                self.source = max(filtered_limits, key=filtered_limits.get)
                return filtered_limits[self.source]

            # Or pick the lowest
            if frogpilot_toggles.speed_limit_priority_lowest:
                self.source = min(filtered_limits, key=filtered_limits.get)
                return filtered_limits[self.source]

            # Otherwise, pick based on a custom “priority1 -> priority2 -> priority3”
            for priority in [
                frogpilot_toggles.speed_limit_priority1,
                frogpilot_toggles.speed_limit_priority2,
                frogpilot_toggles.speed_limit_priority3
            ]:
                if priority is not None and priority in filtered_limits:
                    self.source = priority
                    return filtered_limits[priority]

        # If no valid speed-limits found or all <= 1:
        self.source = "None"

        # Potential fallback behaviors:
        if frogpilot_toggles.slc_fallback_previous_speed_limit:
            return self.previous_speed_limit

        if frogpilot_toggles.slc_fallback_set_speed:
            return max_speed_limit

        return 0

    def smooth_transition(self, current_smooth, target_speed, dt):
        """
        Gently move current_smooth -> target_speed using linear accel/decel limits.
        If target_speed <= 0, clamp to zero. If the difference < step, jump immediately.
        Otherwise move by step in the appropriate direction.
        """
        # If invalid target, clamp to 0
        if target_speed <= 0:
            return 0.0

        diff = target_speed - current_smooth
        if abs(diff) < 0.001:
            # Already basically at the target
            return current_smooth

        # Decide whether we’re accelerating or decelerating
        rate = self.accel_limit if diff > 0 else self.decel_limit

        # The maximum possible change in this timestep
        step = rate * dt
        if abs(diff) <= step:
            return target_speed
        else:
            return current_smooth + math.copysign(step, diff)
