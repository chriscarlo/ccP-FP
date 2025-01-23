#!/usr/bin/env python3

import json
import math
import os
import subprocess
import time
from math import cos, pi, radians, sin, sqrt
from typing import Optional

import requests

from cereal import log, messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.frogpilot.frogpilot_variables import get_frogpilot_toggles

# Path to your Mapbox API credentials file
MAPBOX_API_FILE = "/persist/mapbox/mapbox_api.txt"
MAPBOX_HOST = "https://api.mapbox.com"

# Path to mapd binary
MAPD_PATH = "/data/media/0/osm/mapd"

def load_mapbox_keys() -> tuple[Optional[str], Optional[str]]:
  """
  Load Mapbox API keys from the persist file: expects JSON like:
    {
      "public_key": "pk.xxx",
      "secret_key": "sk.xxx"
    }
  """
  try:
    if os.path.exists(MAPBOX_API_FILE):
      with open(MAPBOX_API_FILE, 'r') as f:
        keys = json.load(f)
        return keys.get("public_key"), keys.get("secret_key")
  except Exception as e:
    cloudlog.exception(f"Failed to load Mapbox API keys: {e}")
  return None, None

def maxspeed_to_ms(maxspeed: dict) -> float:
  """
  Convert a Mapbox maxspeed dict into m/s. Example maxspeed dict:
    { "speed": 35, "unit": "mph" }
  Return 0.0 if invalid.
  """
  if not maxspeed or 'speed' not in maxspeed or 'unit' not in maxspeed:
    return 0.0

  speed = float(maxspeed['speed'])
  unit = maxspeed['unit']

  if unit == 'km/h':
    return speed * (1000.0/3600.0)  # 0.27778...
  elif unit == 'mph':
    return speed * 0.44704
  else:
    # Unknown unit -> fallback 0 or attempt a guess
    return 0.0

def project_coordinate(lat: float, lon: float, heading_deg: float, distance_m: float = 100.0):
  """
  Given a lat/lon in degrees and a heading in degrees, project a second lat/lon
  'distance_m' meters ahead. Returns (lat2, lon2).
  """
  # 1 degree of latitude ~ 111111 m
  # 1 degree of longitude ~ 111111 m * cos(latitude)
  # Convert heading to radians
  heading_rad = radians(heading_deg)

  # Convert distance to degrees
  d_lat = (distance_m / 111111.0) * math.cos(heading_rad)
  # Adjust for smaller distance in the longitude direction
  d_lon = (distance_m / (111111.0 * abs(math.cos(radians(lat))))) * math.sin(heading_rad)

  lat2 = lat + d_lat
  lon2 = lon + d_lon
  return lat2, lon2

class SpeedLimitService:
  def __init__(self, sm=None, pm=None):
    # SubMaster for reading vehicle location
    self.sm = sm or messaging.SubMaster(['liveLocationKalman'])
    # PubMaster for broadcasting speed limit
    self.pm = pm or messaging.PubMaster(['frogpilotNavigation'])

    self.params = Params()

    # Load API keys
    self.public_key, self.secret_key = load_mapbox_keys()
    if not self.secret_key:
      cloudlog.error("No Mapbox secret key found -- speed limit data will be 0")

    # Tracking
    self.last_lat = 0.0
    self.last_lon = 0.0
    self.last_query_time = 0.0
    self.last_speed_limit = 0.0
    self.localizer_valid = False

    # We only query if we've moved > X meters AND it's been > Y seconds
    self.MIN_DISTANCE_CHANGE = 25.0   # meters
    self.MIN_QUERY_INTERVAL = 2.0     # seconds

  def is_mapd_available(self) -> bool:
    """Check if mapd binary exists and is executable."""
    return os.path.isfile(MAPD_PATH) and os.access(MAPD_PATH, os.X_OK)

  def should_query_api(self, lat: float, lon: float) -> bool:
    """Check if enough distance & time have passed since our last query."""
    now = time.monotonic()

    # Always query if no prior query
    if self.last_query_time == 0.0:
      return True

    # Too soon?
    if now - self.last_query_time < self.MIN_QUERY_INTERVAL:
      return False

    # Check distance
    dlat = (lat - self.last_lat) * 111111.0
    dlon = (lon - self.last_lon) * 111111.0 * abs(math.cos(lat * pi / 180.0))
    distance = sqrt(dlat*dlat + dlon*dlon)

    return distance > self.MIN_DISTANCE_CHANGE

  def query_speed_limit(self, lat: float, lon: float, heading_deg: float) -> float:
    """
    Query Mapbox Directions for a route from (lat, lon) to a second point
    ~100m in front of us. Return speed limit in m/s if found, else 0.0.
    """
    if not self.secret_key:
      return 0.0

    # Build second coordinate
    lat2, lon2 = project_coordinate(lat, lon, heading_deg, distance_m=100.0)

    # Build directions API URL: must have two coordinate pairs "lon,lat;lon2,lat2"
    # Use a driving profile. Request the 'maxspeed' annotation.
    url = f"{MAPBOX_HOST}/directions/v5/mapbox/driving/{lon},{lat};{lon2},{lat2}"
    params = {
      'access_token': self.secret_key,
      'annotations': 'maxspeed',
    }

    try:
      resp = requests.get(url, params=params, timeout=5)
      if resp.status_code != 200:
        cloudlog.error(f"Mapbox speed limit query failed with code {resp.status_code}")
        return 0.0

      data = resp.json()
      routes = data.get('routes', [])
      if len(routes) == 0:
        return 0.0

      # We only look at the first route
      route = routes[0]
      legs = route.get('legs', [])
      if len(legs) == 0:
        return 0.0

      annotation = legs[0].get('annotation', {})
      maxspeeds = annotation.get('maxspeed', [])
      if not maxspeeds:
        return 0.0

      # Typically, maxspeed is an array of dicts, one per route segment
      ms = maxspeeds[0]
      if 'unknown' in str(ms).lower() or 'none' in str(ms).lower():
        return 0.0

      return maxspeed_to_ms(ms)
    except Exception as e:
      cloudlog.exception(f"Speed limit query failed: {e}")
      return 0.0

  def query_osm_speed_limit(self, lat: float, lon: float) -> float:
    """
    Query local OSM data for speed limit at current coordinates.
    Returns speed limit in m/s if found, else 0.0.
    """
    if not self.is_mapd_available() or not self.localizer_valid:
      return 0.0

    try:
      # Call mapd binary with current coordinates
      cmd = [MAPD_PATH, "speed_limit", f"{lat}", f"{lon}"]
      result = subprocess.run(cmd, capture_output=True, text=True, timeout=1.0)

      if result.returncode != 0:
        return 0.0

      # Parse speed limit from output
      # mapd outputs: speed_limit_value unit
      # e.g. "45 mph" or "50 km/h"
      output = result.stdout.strip().split()
      if len(output) != 2:
        return 0.0

      try:
        speed = float(output[0])
        unit = output[1]

        # Convert to m/s
        if unit == "mph":
          return speed * 0.44704
        elif unit == "km/h":
          return speed * 0.27778
        else:
          return 0.0

      except ValueError:
        return 0.0

    except Exception as e:
      cloudlog.exception(f"OSM speed limit query failed: {e}")
      return 0.0

  def update(self) -> None:
    """Main update loop: read location, maybe query Mapbox, then broadcast speed limit."""
    self.sm.update(0)
    location = self.sm['liveLocationKalman']
    self.localizer_valid = (location.status == log.LiveLocationKalman.Status.valid)

    if self.localizer_valid:
      lat = location.positionGeodetic.value[0]
      lon = location.positionGeodetic.value[1]
      heading_deg = getattr(location, 'bearingDeg', None)

      # If bearing is None or NaN, calculate from velocity
      if heading_deg is None or math.isnan(heading_deg):
        if hasattr(location, 'vNED') and len(location.vNED) >= 2:
          v_north = location.vNED[0]
          v_east = location.vNED[1]
          if abs(v_north) > 0.1 or abs(v_east) > 0.1:  # Only calculate if moving
            heading_rad = math.atan2(v_east, v_north)  # 0 = north, pi/2 = east
            heading_deg = math.degrees(heading_rad)
            if heading_deg < 0:
              heading_deg += 360.0
          else:
            heading_deg = 0.0  # Default when stationary
        else:
          heading_deg = 0.0  # Default if no velocity data

      if self.should_query_api(lat, lon):
        mapbox_speed_limit = self.query_speed_limit(lat, lon, heading_deg)
        if mapbox_speed_limit > 0.0:
          self.last_speed_limit = mapbox_speed_limit
        self.last_lat = lat
        self.last_lon = lon
        self.last_query_time = time.monotonic()

      # Query OSM data every time (it's local and just needs current position)
      osm_speed_limit = self.query_osm_speed_limit(lat, lon)

      # Publish on frogpilotNavigation
      msg = messaging.new_message('frogpilotNavigation', valid=True)
      if self.last_speed_limit > 0.0 and self.localizer_valid:
        msg.frogpilotNavigation.navigationSpeedLimitRealtime = float(self.last_speed_limit)
      if osm_speed_limit > 0.0:
        msg.frogpilotNavigation.mapSpeedLimitRealtime = float(osm_speed_limit)
      else:
        msg.valid = False

      self.pm.send('frogpilotNavigation', msg)

      # Store in Params in m/s
      self.params.put("MapSpeedLimit", str(self.last_speed_limit))

def main():
  service = SpeedLimitService()
  rk = Ratekeeper(2.0, print_delay_threshold=None)

  while True:
    service.update()
    rk.keep_time()

if __name__ == "__main__":
  main()
