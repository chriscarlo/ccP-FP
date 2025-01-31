#!/usr/bin/env python3

import json
import math
import os
import time
from math import radians, pi, sqrt
from typing import Optional

import requests

from cereal import log, messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog

# Path to your Mapbox API credentials file
MAPBOX_API_FILE = "/persist/mapbox/mapbox_api.txt"
MAPBOX_HOST = "https://api.mapbox.com"

def load_mapbox_keys() -> tuple[Optional[str], Optional[str]]:
    """Load Mapbox API keys from persistent storage"""
    try:
        if os.path.exists(MAPBOX_API_FILE):
            with open(MAPBOX_API_FILE, 'r') as f:
                keys = json.load(f)
                return keys.get("public_key"), keys.get("secret_key")
        cloudlog.error("Mapbox API file not found at %s", MAPBOX_API_FILE)
    except json.JSONDecodeError:
        cloudlog.error("Invalid JSON format in Mapbox API file")
    except Exception as e:
        cloudlog.error("Failed to load Mapbox keys: %s", e)
    return None, None

def maxspeed_to_ms(maxspeed: dict) -> float:
    """Convert Mapbox maxspeed dict to m/s with data validation"""
    if not maxspeed or 'speed' not in maxspeed:
        return 0.0

    try:
        speed = float(maxspeed['speed'])
        unit = maxspeed.get('unit', 'km/h').lower()
    except (TypeError, ValueError):
        cloudlog.debug("Invalid speed value: %s", maxspeed.get('speed'))
        return 0.0

    conversion_factors = {
        'km/h': 0.27778,
        'mph': 0.44704
    }

    return speed * conversion_factors.get(unit, 0.0)

def project_coordinate(lat: float, lon: float, heading_deg: float, distance_m: float = 100.0):
  """
  Given a lat/lon in degrees and a heading in degrees, project a second lat/lon
  'distance_m' meters ahead. Returns (lat2, lon2).
  """
  heading_rad = radians(heading_deg)

  # 1 degree of latitude ~ 111111 m
  # 1 degree of longitude ~ 111111 m * cos(latitude)
  d_lat = (distance_m / 111111.0) * math.cos(heading_rad)
  d_lon = (distance_m / (111111.0 * abs(math.cos(lat * pi / 180.0)))) * math.sin(heading_rad)

  lat2 = lat + d_lat
  lon2 = lon + d_lon
  return lat2, lon2

class SpeedLimitService:
  """
  A service that periodically:
    1. Reads the vehicle's current location and heading.
    2. Pulls the mapd speed limit from /dev/shm/params.
    3. Queries Mapbox for an additional speed limit reading (throttled).
    4. Publishes both values on 'frogpilotNavigation'.
  """
  def __init__(self, sm=None, pm=None):
    # SubMaster for reading vehicle location
    from cereal import messaging
    self.sm = sm or messaging.SubMaster(['liveLocationKalman'])
    # PubMaster for broadcasting speed limit
    self.pm = pm or messaging.PubMaster(['frogpilotNavigation'])

    # Standard param store for general usage (e.g. load_mapbox_keys())
    self.params = Params()
    # Real-time param store for retrieving mapd speed limit
    self.params_memory = Params("/dev/shm/params")

    # Load API keys
    self.public_key, self.secret_key = load_mapbox_keys()
    if not self.secret_key:
      cloudlog.error("No Mapbox secret key found -- Mapbox speed limit will remain 0")

    # Keep track of the last valid Mapbox speed limit to keep publishing it
    # until we can fetch a new one.
    self.last_mapbox_speed_limit = 0.0

    # Keep track of our last location/time to throttle Mapbox queries
    self.last_lat = 0.0
    self.last_lon = 0.0
    self.last_query_time = 0.0
    self.localizer_valid = False

    # Distance/time thresholds for Mapbox queries
    self.MIN_DISTANCE_CHANGE = 25.0   # meters
    self.MIN_QUERY_INTERVAL = 2.0     # seconds

  def should_query_mapbox(self, lat: float, lon: float) -> bool:
    """
    Decide if we should query Mapbox based on distance/time since last query.
    """
    now = time.monotonic()
    if self.last_query_time == 0.0:
      return True

    if now - self.last_query_time < self.MIN_QUERY_INTERVAL:
      return False

    dlat = (lat - self.last_lat) * 111111.0
    dlon = (lon - self.last_lon) * 111111.0 * abs(math.cos(lat * pi / 180.0))
    distance = sqrt(dlat * dlat + dlon * dlon)

    return distance > self.MIN_DISTANCE_CHANGE

  def query_mapbox_speed_limit(self, lat: float, lon: float, heading_deg: float) -> float:
    """Query Mapbox Directions API for current speed limit"""
    if not self.secret_key:
        cloudlog.debug("Mapbox secret key not available")
        return 0.0

    # Project 1000 meters ahead for better API results
    lat2, lon2 = project_coordinate(lat, lon, heading_deg, 1000.0)

    try:
        url = f"{MAPBOX_HOST}/directions/v5/mapbox/driving/{lon},{lat};{lon2},{lat2}"
        params = {
            'access_token': self.secret_key,
            'annotations': 'maxspeed',
            'overview': 'full'  # Required parameter for speed data
        }

        resp = requests.get(url, params=params, timeout=3)
        resp.raise_for_status()

        data = resp.json()
        route = data.get('routes', [{}])[0]
        leg = route.get('legs', [{}])[0]
        annotation = leg.get('annotation', {})
        maxspeeds = annotation.get('maxspeed', [])

        if not maxspeeds:
            return 0.0

        # Handle API returning 'unknown' as first value
        valid_speeds = [ms for ms in maxspeeds if not isinstance(ms, str) or ms.lower() not in ('unknown', 'none')]
        return maxspeed_to_ms(valid_speeds[0]) if valid_speeds else 0.0

    except requests.exceptions.RequestException as e:
        cloudlog.debug("Mapbox API request failed: %s", e)
    except json.JSONDecodeError:
        cloudlog.debug("Invalid JSON response from Mapbox")

    return 0.0

  def update(self) -> None:
    """
    Main update loop: read location, retrieve mapd speed limit,
    query Mapbox (if appropriate), then broadcast both values.
    """
    self.sm.update(0)
    location = self.sm['liveLocationKalman']
    self.localizer_valid = (location.status == log.LiveLocationKalman.Status.valid)

    mapd_speed_limit = 0.0
    mapbox_speed_limit = self.last_mapbox_speed_limit

    if self.localizer_valid:
      lat = location.positionGeodetic.value[0]
      lon = location.positionGeodetic.value[1]
      heading_deg = getattr(location, 'bearingDeg', None)

      # If bearing is None or NaN, approximate from velocity
      if heading_deg is None or math.isnan(heading_deg):
        if hasattr(location, 'vNED') and len(location.vNED) >= 2:
          v_north = location.vNED[0]
          v_east = location.vNED[1]
          # Only if moving
          if abs(v_north) > 0.1 or abs(v_east) > 0.1:
            heading_rad = math.atan2(v_east, v_north)
            heading_deg = math.degrees(heading_rad)
            if heading_deg < 0:
              heading_deg += 360.0
          else:
            heading_deg = 0.0
        else:
          heading_deg = 0.0

      # 1) Always read the real-time mapd speed limit from /dev/shm/params
      mapd_speed_limit = self.params_memory.get_float("MapSpeedLimit")

      # 2) If the time/distance threshold is met, query Mapbox for a fresh limit
      if self.should_query_mapbox(lat, lon):
        fresh_mapbox_limit = self.query_mapbox_speed_limit(lat, lon, heading_deg)
        # If we got a valid speed limit from Mapbox, update the stored value
        if fresh_mapbox_limit > 0.0:
          mapbox_speed_limit = fresh_mapbox_limit
          self.last_mapbox_speed_limit = fresh_mapbox_limit

        self.last_lat = lat
        self.last_lon = lon
        self.last_query_time = time.monotonic()

    # 3) Publish both speed limits on frogpilotNavigation
    msg = messaging.new_message('frogpilotNavigation', valid=True)
    msg.frogpilotNavigation.mapSpeedLimitRealtime = float(mapd_speed_limit)
    msg.frogpilotNavigation.navigationSpeedLimitRealtime = float(mapbox_speed_limit)

    self.pm.send('frogpilotNavigation', msg)

def main():
  service = SpeedLimitService()
  rk = Ratekeeper(2.0, print_delay_threshold=None)

  while True:
    service.update()
    rk.keep_time()

if __name__ == "__main__":
  main()
