import math

from openpilot.common.numpy_fast import clip  # We still use clip for safety
from openpilot.selfdrive.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.selfdrive.controls.lib.longitudinal_planner import A_CRUISE_MIN, get_max_accel

from openpilot.selfdrive.frogpilot.frogpilot_variables import CITY_SPEED_LIMIT

# Original, but we keep them for reference or fallback (especially if other parts of the code import them).
A_CRUISE_MIN_ECO =   A_CRUISE_MIN / 2
A_CRUISE_MIN_SPORT = A_CRUISE_MIN * 2

###############################################################################
# Logistic helper:
#   lower + (upper - lower) / (1 + e^(-scale * (x - midpoint)))
###############################################################################
def logistic(x, lower, upper, midpoint, scale):
  """
  Smoothly transitions from 'lower' to 'upper' around 'midpoint' with steepness 'scale'.
  """
  return lower + (upper - lower) / (1.0 + math.exp(-scale * (x - midpoint)))


###############################################################################
# 1. Max accel: ECO, Sport, Sport+
#    Previously, piecewise breakpoints; now replaced with smoother transitions.
#    We do an internal mph conversion if the old breakpoints were indeed mph.
#    Adjust the parameters as needed to get the desired shape.
###############################################################################
def get_max_accel_eco(v_ego):
  """
  Replaces the piecewise 'interp' for ECO mode with a single logistic function.
  The shape starts higher at low speed and gently tapers at higher speeds.
  """
  v_mph = v_ego * 2.236936  # Convert from m/s to mph if your code used mph breakpoints
  # Approximate the original shape: from ~2.0 at 0 mph down to ~0.2 at 40 mph.
  return logistic(
      x=v_mph,
      lower=0.2,     # final taper
      upper=2.0,     # initial maximum
      midpoint=20.0, # center of transition
      scale=0.15     # steeper or gentler slope
  )


def get_max_accel_sport(v_ego):
  """
  Sport mode: slightly higher acceleration than ECO.
  """
  v_mph = v_ego * 2.236936
  # Approximate from ~3.0 at 0 mph down to ~0.6–1.0 at higher speeds
  return logistic(
      x=v_mph,
      lower=0.8,     # a bit higher final taper than ECO
      upper=3.0,     # initial maximum
      midpoint=20.0,
      scale=0.15
  )


def get_max_accel_sport_plus(v_ego):
  """
  Sport+ mode: even higher performance, yet still smoothly transitions.
  """
  v_mph = v_ego * 2.236936
  # Approximate from ~4.0 at 0 mph down to ~1.0 at high speeds
  return logistic(
      x=v_mph,
      lower=1.0,
      upper=4.0,
      midpoint=20.0,
      scale=0.15
  )


###############################################################################
# 2. Low-speed accel: transitions from fraction of max_accel to full max_accel
#    as v_cruise goes from 0 to city limit. We use a smooth function here too.
###############################################################################
def get_max_accel_low_speeds(max_accel, v_cruise):
  """
  Emulates a pro driver gently modulating throttle at low speeds,
  starting around max_accel/4 near zero mph, up to max_accel by city speed limit.
  """
  # For the transition, we treat v_cruise from 0 to CITY_SPEED_LIMIT as [0, 1] scaled input
  # to a logistic function. That ensures a gentle rise.
  if CITY_SPEED_LIMIT <= 0:
    return max_accel  # fallback if city speed limit is zero or invalid

  fraction = clip(v_cruise / CITY_SPEED_LIMIT, 0.0, 1.0)
  lower = max_accel / 4.0
  upper = max_accel

  # We'll center the logistic around fraction=0.5
  return logistic(
      x=fraction,
      lower=lower,
      upper=upper,
      midpoint=0.5,
      scale=6.0  # tweak for how quickly we want to ramp up
  )


###############################################################################
# 3. Ramp-off: smoothly reduce acceleration if the setpoint is close to current
#    speed (v_cruise - v_ego). Replaces piecewise [0., 1., 5., 10.].
###############################################################################
def get_max_accel_ramp_off(max_accel, v_cruise, v_ego):
  """
  If v_cruise is only slightly higher than v_ego, we want to softly ramp
  from near 0 to max_accel as the difference (v_cruise - v_ego) grows.
  """
  diff = max(v_cruise - v_ego, 0.0)
  # For small differences, accelerate less; for large differences, accelerate fully.
  # We'll consider ~10 m/s difference as "definitely need full accel".
  # Using a logistic shape: from near 0 at diff=0 to near 1 at diff=10.
  fraction = logistic(
      x=diff,
      lower=0.0,
      upper=1.0,
      midpoint=5.0,  # center of transition
      scale=0.6
  )
  return fraction * max_accel


###############################################################################
# 4. Max allowed accel per ISO 15622:2018: we keep the same shape but can
#    smooth it slightly using logistic if desired. Here, we'll do it gently.
###############################################################################
def get_max_allowed_accel(v_ego):
  """
  The original piecewise was:
      [0., 5., 20.] -> [4.0, 4.0, 2.0]
  We'll keep the same range but use a gentle logistic so it doesn't feel abrupt.
  """
  v_mph = v_ego * 2.236936
  # From 4.0 at or near 0 mph, gently down to 2.0 by ~20 mph.
  # We'll allow a small plateau between 0 and 5 mph.
  # A trick is to do a two-stage shape or just clamp with logistic and a plateau.
  # For simplicity, a single logistic that starts near 4 for v_mph < 5 and ends near 2 for v_mph > 20.
  # We'll artificially shift the midpoint up a bit so we remain near 4.0 for a while.
  base_val = logistic(
      x=v_mph,
      lower=2.0,
      upper=4.0,
      midpoint=12.5,  # pick midpoint between 5 and 20
      scale=0.3
  )
  # Then clamp so that at very low speeds it doesn't dip below 4.0
  return max(base_val, 4.0 if v_mph <= 5 else 2.0)


###############################################################################
# 5. The main FrogPilotAcceleration class, lightly updated for new function calls
###############################################################################
class FrogPilotAcceleration:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    self.max_accel = 0
    self.min_accel = 0

  def update(self, controlsState, frogpilotCarState, v_ego, frogpilot_toggles):
    eco_gear = frogpilotCarState.ecoGear
    sport_gear = frogpilotCarState.sportGear

    # Decide on base max_accel
    if frogpilotCarState.trafficModeActive:
      # fallback to openpilot standard get_max_accel
      self.max_accel = get_max_accel(v_ego)
    elif frogpilot_toggles.map_acceleration and (eco_gear or sport_gear):
      # Eco or Sport from map-based triggers
      if eco_gear:
        self.max_accel = get_max_accel_eco(v_ego)
      else:
        if frogpilot_toggles.acceleration_profile == 3:
          self.max_accel = get_max_accel_sport_plus(v_ego)
        else:
          self.max_accel = get_max_accel_sport(v_ego)
    else:
      # Manual user profiles
      if frogpilot_toggles.acceleration_profile == 1:
        self.max_accel = get_max_accel_eco(v_ego)
      elif frogpilot_toggles.acceleration_profile == 2:
        self.max_accel = get_max_accel_sport(v_ego)
      elif frogpilot_toggles.acceleration_profile == 3:
        self.max_accel = get_max_accel_sport_plus(v_ego)
      elif controlsState.experimentalMode:
        self.max_accel = ACCEL_MAX
      else:
        self.max_accel = get_max_accel(v_ego)

    # Additional smoothing if human_acceleration toggle is on
    if frogpilot_toggles.human_acceleration:
      if self.frogpilot_planner.frogpilot_following.following_lead and not frogpilotCarState.trafficModeActive:
        # Clip lead-based acceleration between an aggressive lower bound and a standard upper
        # We'll assume lead_one.aLeadK is already computed
        self.max_accel = clip(
          self.frogpilot_planner.lead_one.aLeadK,
          get_max_accel_sport_plus(v_ego),
          get_max_allowed_accel(v_ego)
        )

      # Fine-tune max_accel for low speeds
      self.max_accel = min(
        get_max_accel_low_speeds(self.max_accel, self.frogpilot_planner.v_cruise),
        self.max_accel
      )
      # Ramp off if we’re close to the setpoint
      self.max_accel = min(
        get_max_accel_ramp_off(self.max_accel, self.frogpilot_planner.v_cruise, v_ego),
        self.max_accel
      )

    # Decide on min_accel
    if controlsState.experimentalMode:
      self.min_accel = ACCEL_MIN
    elif frogpilot_toggles.map_deceleration and (eco_gear or sport_gear):
      if eco_gear:
        self.min_accel = A_CRUISE_MIN_ECO
      else:
        self.min_accel = A_CRUISE_MIN_SPORT
    else:
      if frogpilot_toggles.deceleration_profile == 1:
        self.min_accel = A_CRUISE_MIN_ECO
      elif frogpilot_toggles.deceleration_profile == 2:
        self.min_accel = A_CRUISE_MIN_SPORT
      else:
        self.min_accel = A_CRUISE_MIN
