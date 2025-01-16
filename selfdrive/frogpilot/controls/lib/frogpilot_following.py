import math
from openpilot.common.numpy_fast import clip, interp
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import (
    COMFORT_BRAKE,
    STOP_DISTANCE,
    get_jerk_factor,
    get_safe_obstacle_distance,
    get_stopped_equivalence_factor,
    get_T_FOLLOW,
    LongitudinalMpc,
)
from openpilot.selfdrive.frogpilot.frogpilot_variables import CITY_SPEED_LIMIT, CRUISING_SPEED

TRAFFIC_MODE_BP = [0.0, CITY_SPEED_LIMIT]

def logistic_interpolate(x, x_min, x_max, y_min, y_max, slope=1.0):
  """
  Logistic-based interpolation between (x_min -> y_min) and (x_max -> y_max).
  'slope' controls how quickly we transition near the midpoint.

  We'll use this to gently reduce jerk at higher speeds (so we don't "slam"
  on gas/brakes), but still allow immediate transitions from decel to accel.
  """
  slope = max(abs(slope), 1e-6)  # avoid degenerate slopes
  x_clamped = clip(x, x_min, x_max)
  midpoint = (x_min + x_max) / 2.0
  return y_min + (y_max - y_min) / (1.0 + math.e ** (-slope * (x_clamped - midpoint)))


class FrogPilotFollowing:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner

    # State flags
    self.following_lead = False
    self.slower_lead = False

    # Jerk/follow params
    self.acceleration_jerk = 0.0
    self.base_acceleration_jerk = 0.0
    self.base_danger_jerk = 0.0
    self.base_speed_jerk = 0.0
    self.danger_jerk = 0.0
    self.speed_jerk = 0.0
    self.t_follow = 1.45  # default placeholder, overridden by toggles/OP calls

    # For reference by other parts of code
    self.safe_obstacle_distance = 0
    self.safe_obstacle_distance_stock = 0
    self.stopped_equivalence_factor = 0

  def update(self, aEgo, controlsState, frogpilotCarState, lead_distance, v_ego, v_lead, frogpilot_toggles):
    """
    Main update:
      - Pick base jerk from toggles or OP personalities
      - Gently scale jerk by speed (to avoid "slams" at highway speeds)
      - Keep immediate transitions from decel -> accel (no artificial lag)
      - Let MPC handle precise approach; we keep t_follow stable
    """

    # --------------------------
    # 1. Pick base jerk & t_follow
    # --------------------------
    if frogpilotCarState.trafficModeActive:
      # Traffic mode => toggles-based interpolation
      if aEgo >= 0:
        # Acceleration
        self.base_acceleration_jerk = interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_acceleration)
        self.base_speed_jerk = interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_speed)
      else:
        # Deceleration
        self.base_acceleration_jerk = interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_deceleration)
        self.base_speed_jerk = interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_speed_decrease)

      self.base_danger_jerk = interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_danger)
      self.t_follow = interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_follow)

    else:
      # Non-traffic mode => personalities, get_jerk_factor, get_T_FOLLOW
      if aEgo >= 0:
        # Accel
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
        # Decel
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

      self.t_follow = get_T_FOLLOW(
        frogpilot_toggles.aggressive_follow,
        frogpilot_toggles.standard_follow,
        frogpilot_toggles.relaxed_follow,
        frogpilot_toggles.custom_personalities,
        controlsState.personality
      )

    # --------------------------
    # 2. Scale jerk gently with speed
    # --------------------------
    jerk_scale = logistic_interpolate(
      x=v_ego,
      x_min=0.0,    # 0 m/s
      x_max=40.0,   # ~90 mph
      y_min=1.05,   # mild boost at low speeds
      y_max=0.85,   # mild reduction at high speeds
      slope=0.7     # not too steep
    )
    self.acceleration_jerk = self.base_acceleration_jerk * jerk_scale
    self.danger_jerk = self.base_danger_jerk * jerk_scale
    self.speed_jerk = self.base_speed_jerk * jerk_scale

    # --------------------------
    # 3. Lead detection & distances
    # --------------------------
    # Basic flag to say "We have a lead that's within a relevant distance."
    self.following_lead = (
      self.frogpilot_planner.tracking_lead
      and (lead_distance < (self.t_follow + 1.0) * v_ego)
    )

    if self.frogpilot_planner.tracking_lead:
      self.safe_obstacle_distance = int(get_safe_obstacle_distance(v_ego, self.t_follow))
      self.safe_obstacle_distance_stock = self.safe_obstacle_distance
      self.stopped_equivalence_factor = int(get_stopped_equivalence_factor(v_lead))
      # Small adjustments for faster/slower lead without messing up t_follow
      self.update_follow_values(lead_distance, v_ego, v_lead, frogpilot_toggles)
    else:
      self.safe_obstacle_distance = 0
      self.safe_obstacle_distance_stock = 0
      self.stopped_equivalence_factor = 0

    # ---------------------------
    # (NEW) Optionally push these jerk factors & t_follow into the ACC MPC
    # for "Dr. Limo / Mr. Andretti"
    # Example usage if you have an mpc instance and want to update it:
    # (Be sure the mpc is in ACC mode.)
    # Safely reference the MPC (if it exists) for Dr. Limo / Mr. Andretti logic
    mpc = getattr(self.frogpilot_planner, 'mpc', None)
    if mpc and mpc.mode == 'acc':
      mpc.params[:,4] = self.t_follow  # update T_FOLLOW
      mpc.set_weights(
        acceleration_jerk=self.acceleration_jerk,
        danger_jerk=self.danger_jerk,
        speed_jerk=self.speed_jerk,
        prev_accel_constraint=True,
        personality=controlsState.personality,
        speed_scaling=1.0
      )
    # ---------------------------

  def update_follow_values(self, lead_distance, v_ego, v_lead, frogpilot_toggles):
    """
    Fine-tune how we handle faster vs. slower lead, *without* large changes to t_follow.
    - We keep t_follow as the primary target, trusting the MPC to do the "perfect slide."
    - We do small, incremental jerk tweaks so we don't slam on gas/brakes.
    - If we overshoot or undershoot a bit, let it happen without freaking out.
    """

    # We'll define a small margin around t_follow * v_ego to allow slight drift
    # before making *tiny* jerk adjustments. This prevents "freak-outs" but also
    # stops us from yoyoyo-lurching if the lead changes speed quickly.
    follow_target = self.t_follow * v_ego
    margin_frac = 0.05  # e.g. ±5% of ideal distance
    lower_bound = follow_target * (1.0 - margin_frac)
    upper_bound = follow_target * (1.0 + margin_frac)

    if v_lead < v_ego:
      # Lead is slower => avoid slamming brakes if we get a bit close,
      # but do small incremental "cushion."
      # We'll do a gentle decel jerk tweak if we're significantly below lower_bound.
      if lead_distance < lower_bound:
        # TINY bump to decel jerk => more gentle braking (not a slam).
        self.acceleration_jerk *= 0.95
        self.speed_jerk *= 0.95
      self.slower_lead = True
    else:
      self.slower_lead = False

    if v_lead > v_ego:
      # Lead is faster => we do not want to reduce t_follow (which would cause a big push).
      # Instead, if we're above upper_bound, it means the lead is pulling away and
      # we have a bigger gap than intended. We'll lightly increase acceleration jerk
      # so we catch up smoothly—NOT a slam.
      if lead_distance > upper_bound:
        self.acceleration_jerk *= 1.05
        self.speed_jerk *= 1.05

    # If lead_distance is within ±5% around t_follow * v_ego, we do nothing special:
    # let the MPC run and keep it stable.
