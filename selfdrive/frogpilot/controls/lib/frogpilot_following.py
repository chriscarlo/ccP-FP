import math

from openpilot.common.numpy_fast import clip, interp
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import COMFORT_BRAKE, STOP_DISTANCE, get_jerk_factor, get_safe_obstacle_distance, get_stopped_equivalence_factor, get_T_FOLLOW

from openpilot.selfdrive.frogpilot.frogpilot_variables import CITY_SPEED_LIMIT, CRUISING_SPEED

TRAFFIC_MODE_BP = [0., CITY_SPEED_LIMIT]

def logistic(x):
  return 1.0 / (1.0 + math.exp(-x))

def calc_stability_factor(v_ego, v_lead, lead_distance, base_dist, stable_v_thresh=0.3, stable_d_thresh=2.0):
  """
  Returns a factor in [0.0, 1.0] indicating how stable the follow scenario is.
  - A higher value (closer to 1) means the ego and lead are closely matched in speed
    and distance, and there's no strong reason to deviate from the baseline time gap.
  - A lower value (closer to 0) indicates bigger speed or distance mismatches.
  """
  dv = abs(v_ego - v_lead)
  dd = abs(lead_distance - base_dist)

  measure = (dv / max(stable_v_thresh, 1e-9))**0.5 + (dd / max(stable_d_thresh, 1e-9))**0.5
  stability_factor = 1.0 / (1.0 + measure**2)
  return clip(stability_factor, 0.0, 1.0)

class FrogPilotFollowing:
  def __init__(self, FrogPilotPlanner):
    self.frogpilot_planner = FrogPilotPlanner
    self.following_lead = False

    # Track whether a lead was present in the previous update cycle
    self.was_lead_tracked_last_cycle = False

    self.acceleration_jerk = 0
    self.base_acceleration_jerk = 0
    self.base_speed_jerk = 0
    self.base_danger_jerk = 0
    self.danger_jerk = 0
    self.safe_obstacle_distance = 0
    self.safe_obstacle_distance_stock = 0
    self.speed_jerk = 0
    self.stopped_equivalence_factor = 0
    self.t_follow = 0

    self.acc_d = 0.0  # Accumulator for distance error
    self.aEgo = 0.0

    self.SLOW_LEAD_RANGE = 0.5
    self.SLOW_LEAD_LOGISTIC_K = 0.4
    self.SLOW_LEAD_X0 = 0.1
    self.SLOW_LEAD_ALPHA_DIST = 0.15
    self.SLOW_LEAD_MIN_T = 1.0
    self.SLOW_LEAD_SMOOTH = 0.15

    self.PREDICTIVE_HORIZON = 1.2
    self.PREDICTIVE_DT = 0.1
    self.PREDICTIVE_DECEL_PENALTY = 1.5

    self.MAX_TF_STEP = 0.05
    self.T_FOLLOW_MAX = 2.5

    self.INTEGRATOR_FACTOR = 0.02

    self.NEG_LEAD_ACCEL_FACTOR = 0.92

  def update(self, aEgo, controlsState, frogpilotCarState, lead_distance, v_ego, v_lead, frogpilot_toggles):
    self.aEgo = aEgo
    if frogpilotCarState.trafficModeActive:
      if aEgo >= 0:
        self.base_acceleration_jerk = interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_acceleration)
        self.base_speed_jerk = interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_speed)
      else:
        self.base_acceleration_jerk = interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_deceleration)
        self.base_speed_jerk = interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_speed_decrease)

      self.base_danger_jerk = interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_jerk_danger)
      self.t_follow = interp(v_ego, TRAFFIC_MODE_BP, frogpilot_toggles.traffic_mode_follow)
    else:
      if aEgo >= 0:
        self.base_acceleration_jerk, self.base_danger_jerk, self.base_speed_jerk = get_jerk_factor(
          frogpilot_toggles.aggressive_jerk_acceleration, frogpilot_toggles.aggressive_jerk_danger, frogpilot_toggles.aggressive_jerk_speed,
          frogpilot_toggles.standard_jerk_acceleration, frogpilot_toggles.standard_jerk_danger, frogpilot_toggles.standard_jerk_speed,
          frogpilot_toggles.relaxed_jerk_acceleration, frogpilot_toggles.relaxed_jerk_danger, frogpilot_toggles.relaxed_jerk_speed,
          frogpilot_toggles.custom_personalities, controlsState.personality
        )
      else:
        self.base_acceleration_jerk, self.base_danger_jerk, self.base_speed_jerk = get_jerk_factor(
          frogpilot_toggles.aggressive_jerk_deceleration, frogpilot_toggles.aggressive_jerk_danger, frogpilot_toggles.aggressive_jerk_speed_decrease,
          frogpilot_toggles.standard_jerk_deceleration, frogpilot_toggles.standard_jerk_danger, frogpilot_toggles.standard_jerk_speed_decrease,
          frogpilot_toggles.relaxed_jerk_deceleration, frogpilot_toggles.relaxed_jerk_danger, frogpilot_toggles.relaxed_jerk_speed_decrease,
          frogpilot_toggles.custom_personalities, controlsState.personality
        )

      self.t_follow = get_T_FOLLOW(
        frogpilot_toggles.aggressive_follow,
        frogpilot_toggles.standard_follow,
        frogpilot_toggles.relaxed_follow,
        frogpilot_toggles.custom_personalities, controlsState.personality
      )

    self.acceleration_jerk = self.base_acceleration_jerk
    self.danger_jerk = self.base_danger_jerk
    self.speed_jerk = self.base_speed_jerk

    self.following_lead = self.frogpilot_planner.tracking_lead and lead_distance < (self.t_follow + 1) * v_ego

    if self.frogpilot_planner.tracking_lead:
      self.safe_obstacle_distance = int(get_safe_obstacle_distance(v_ego, self.t_follow))
      self.safe_obstacle_distance_stock = self.safe_obstacle_distance
      self.stopped_equivalence_factor = int(get_stopped_equivalence_factor(v_lead))
      self.update_follow_values(lead_distance, v_ego, v_lead, frogpilot_toggles, frogpilotCarState, controlsState)
    else:
      self.safe_obstacle_distance = 0
      self.safe_obstacle_distance_stock = 0
      self.stopped_equivalence_factor = 0

  def update_follow_values(self, lead_distance, v_ego, v_lead, frogpilot_toggles, frogpilotCarState, controlsState):
    if not frogpilot_toggles.human_following:
      return

    was_lead_tracked = self.was_lead_tracked_last_cycle
    is_lead_tracked = self.frogpilot_planner.tracking_lead

    # If we just lost track of a lead, reset integrator
    if was_lead_tracked and not is_lead_tracked:
      self.acc_d = 0.0

    self.was_lead_tracked_last_cycle = is_lead_tracked

    if not self.frogpilot_planner.tracking_lead:
      return

    delta_v = v_lead - v_ego
    base_dist = self.t_follow * v_lead
    delta_d = lead_distance - base_dist

    # Accumulator update
    alpha = 0.8
    if delta_d <= 0:
      self.acc_d = 0.0
    else:
      self.acc_d = alpha * self.acc_d + (1.0 - alpha) * delta_d
    self.acc_d = clip(self.acc_d, 0.0, 15.0)

    # -------------------------------------------------------------------------
    # TIME-TO-COLLISION LOGIC
    # -------------------------------------------------------------------------
    if v_ego > v_lead:
      relative_speed = max(v_ego - v_lead, 0.1)
      ttc = lead_distance / relative_speed
      ttc = min(ttc, 30.0)
    else:
      ttc = 30.0

    max_decel = 5.5
    time_to_stop = v_ego / max(max_decel, 0.1)
    safety_margin_factor = 2.0
    ttc_ref = max(time_to_stop * safety_margin_factor, 4.0)

    k_ttc = 3.5
    urgency = logistic(k_ttc * (ttc_ref - ttc))
    urgency = clip(urgency, 0.0, 1.0)

    # -------------------------------------------------------------------------
    # --- NEW LOGIC FOR aLeadK ---
    # Make urgency more sensitive if lead is decelerating, less if accelerating.
    # aLeadK = 0 is baseline.
    # You can tweak decel_scale / accel_scale after testing.
    # -------------------------------------------------------------------------
    raw_lead_accel = self.frogpilot_planner.lead_one.aLeadK if self.frogpilot_planner.tracking_lead else 0.0
    if raw_lead_accel < 0.0:
      decel_scale = 0.15  # Increase urgency for negative acceleration
      urgency += decel_scale * abs(raw_lead_accel)
    else:
      accel_scale = 0.1  # Decrease urgency for positive acceleration
      urgency -= accel_scale * raw_lead_accel

    # Clamp urgency back to [0, 1]
    urgency = clip(urgency, 0.0, 1.0)
    # -------------------------------------------------------------------------

    # *** SIGN FLIPPED HERE ***
    X = (abs(delta_v) - self.SLOW_LEAD_X0) \
        + (self.SLOW_LEAD_ALPHA_DIST * delta_d) \
        - (self.INTEGRATOR_FACTOR * self.acc_d)

    # Existing negative acceleration penalty
    if raw_lead_accel < 0.0:
      X -= (self.NEG_LEAD_ACCEL_FACTOR * abs(raw_lead_accel))

    # We also fold in urgency
    urgency_scale = 4.0
    X -= (urgency_scale * urgency)

    reduce_amount = self.SLOW_LEAD_RANGE * logistic(self.SLOW_LEAD_LOGISTIC_K * X)
    desired_t_follow = self.t_follow - reduce_amount
    desired_t_follow = clip(desired_t_follow, self.SLOW_LEAD_MIN_T, self.t_follow)

    # Pull-away handling
    pull_away_x0 = 0.05
    pull_away_k = 5.0
    pull_away_input = delta_v - pull_away_x0
    pull_away_intensity = logistic(pull_away_k * pull_away_input)
    pull_away_intensity = clip(pull_away_intensity, 0.0, 1.0)

    baseline_smoothing = interp(pull_away_intensity, [0.0, 1.0], [self.SLOW_LEAD_SMOOTH, 1.0])

    if raw_lead_accel < 0.0:
      current_gap_ratio = clip(lead_distance / (v_ego * self.t_follow), 0.5, 1.5)
      adjusted_decel = -raw_lead_accel * (1.0 / current_gap_ratio)
      decel_norm = clip(adjusted_decel, 0.0, 6.0) / 6.0
      response_curve = logistic(decel_norm * 2.0 - 1.0)
      smoothing_floor = 0.10
      adjusted_smooth = smoothing_floor + (baseline_smoothing - smoothing_floor) * (1.0 - response_curve)
      baseline_smoothing = min(baseline_smoothing, adjusted_smooth)

    min_smoothing = 0.10
    dynamic_smoothing = (1.0 - urgency) * baseline_smoothing + urgency * min_smoothing

    old_t_follow = self.t_follow
    self.t_follow = (1.0 - dynamic_smoothing) * self.t_follow + dynamic_smoothing * desired_t_follow

    # Predictive check
    predictive_tf = self.compute_predictive_t_follow(
      current_t_follow=self.t_follow,
      v_ego=v_ego,
      v_lead=v_lead,
      lead_distance=lead_distance,
      desired_t_follow=self.t_follow,
      urgency=urgency
    )

    if predictive_tf > self.t_follow:
      self.t_follow = predictive_tf

    self.t_follow = self.soft_saturate_t_follow(
      self.t_follow,
      self.SLOW_LEAD_MIN_T,
      self.T_FOLLOW_MAX,
      k=1.5
    )

    self.t_follow = self.slew_rate_limit_t_follow(
      self.t_follow,
      old_t_follow,
      self.MAX_TF_STEP
    )

    # Stabilization
    base_dist_stable = self.t_follow * v_ego
    stability_factor = calc_stability_factor(v_ego, v_lead, lead_distance, base_dist_stable)

    if stability_factor > 0.85:
      self.acc_d *= 0.9

    if stability_factor > 0.95 and abs(delta_d) < 1.0:
      self.acc_d = 0.0

    far_lead_offset = max(v_lead - CITY_SPEED_LIMIT, 1)
    logistic_offset = 1.0 + reduce_amount
    self.slower_lead = (logistic_offset / far_lead_offset) > 1

  def compute_predictive_t_follow(self, current_t_follow, v_ego, v_lead, lead_distance,
                                  desired_t_follow=None, urgency=0.0):
    """
    Predicts short-term future distance to the lead over self.PREDICTIVE_HORIZON
    (e.g., 1.0s) in increments of self.PREDICTIVE_DT (e.g., 0.1s). If the
    projected gap gets too small, we nudge t_follow back up for safety.
    """

    # Start from the main logic's desired t_follow (if provided), or current.
    predictive_tf = desired_t_follow if desired_t_follow is not None else current_t_follow

    # Gently raise if there's high urgency (e.g. low TTC, strong lead decel).
    predictive_tf += 0.2 * urgency

    dt = self.PREDICTIVE_DT
    steps = int(self.PREDICTIVE_HORIZON / dt)

    dist = lead_distance
    ve = v_ego
    vl = v_lead

    min_future_dist = dist

    for _ in range(steps):
      # Lead's deceleration from aLeadK, or fallback guess if no lead acceleration data
      lead_decel = self.frogpilot_planner.lead_one.aLeadK if self.frogpilot_planner.tracking_lead else (
        -1.5 if vl < ve else 0.0
      )

      # If lead decel is lower than -0.3, scale up a bit to be safe
      if lead_decel < -0.3:
        neg_decel_norm = clip(-(lead_decel + 0.3), 0.0, 5.0) / 5.0
        # Slightly increase the perceived decel factor to err on the safe side
        decel_scale = interp(neg_decel_norm, [0.0, 1.0], [1.0, 1.2])
        lead_decel *= decel_scale

      # Clamp our own acceleration in simulation
      ego_accel = clip(self.aEgo, -6.0, 4.0)

      vl_next = max(0.0, vl + lead_decel * dt)
      ve_next = max(0.0, ve + ego_accel * dt)

      dist_next = dist + (vl - ve) * dt

      # Track the smallest distance encountered
      if dist_next < min_future_dist:
        min_future_dist = dist_next

      vl = vl_next
      ve = ve_next
      dist = dist_next

    # Compute how much "time gap" we get at the tightest future moment
    ve = max(ve, 0.1)
    predicted_gap = min_future_dist / ve

    # If that future gap is smaller than the predictive_tf, raise t_follow
    if predicted_gap < predictive_tf:
      diff = predictive_tf - predicted_gap

      # Scale how quickly we correct upward based on urgency
      gain_base = 0.5       # baseline correction
      gain_urgency = 0.3 * urgency
      gain = gain_base + gain_urgency

      return current_t_follow + (gain * diff)

    # Otherwise, keep the current t_follow
    return current_t_follow


  def soft_saturate_t_follow(self, t_raw, t_min, t_max, k=2.0):
    """
    Smoothly saturate t_raw between t_min and t_max using a logistic-like approach.
    """
    mid = 0.5 * (t_min + t_max)
    half_range = 0.5 * (t_max - t_min) + 1e-9
    scale_in = (t_raw - mid) / half_range

    sat = (2.0 / (1.0 + math.exp(-k * scale_in))) - 1.0
    t_saturated = sat * half_range + mid
    return t_saturated

  def slew_rate_limit_t_follow(self, t_new, t_old, max_step):
    delta = t_new - t_old
    if abs(delta) > max_step:
      delta = max_step * (delta / abs(delta))
    return t_old + delta
