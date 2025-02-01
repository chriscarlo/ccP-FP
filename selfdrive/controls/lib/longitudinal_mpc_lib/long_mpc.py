#!/usr/bin/env python3
import os
import time
import math
import numpy as np
from cereal import log
from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.modeld.constants import index_function
from openpilot.selfdrive.car.interfaces import ACCEL_MIN
if __name__ == '__main__':
  from openpilot.third_party.acados.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
else:
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython
from casadi import SX, vertcat

MODEL_NAME = 'long'
LONG_MPC_DIR = os.path.dirname(os.path.abspath(__file__))
EXPORT_DIR = os.path.join(LONG_MPC_DIR, "c_generated_code")
JSON_FILE = os.path.join(LONG_MPC_DIR, "acados_ocp_long.json")

SOURCES = ['lead0', 'lead1', 'cruise', 'e2e']

X_DIM = 3
U_DIM = 1
PARAM_DIM = 6
COST_E_DIM = 5
COST_DIM = COST_E_DIM + 1
CONSTR_DIM = 4

X_EGO_OBSTACLE_COST = 3.0
X_EGO_COST = 0.0
V_EGO_COST = 0.0
A_EGO_COST = 0.0
J_EGO_COST = 5.0     # Base jerk cost (further scaled dynamically)
A_CHANGE_COST = 150.0
DANGER_ZONE_COST = 100.0

CRASH_DISTANCE = 0.25
LEAD_DANGER_FACTOR = 0.75

LIMIT_COST = 1e6

ACADOS_SOLVER_TYPE = 'SQP_RTI'

LEAD_ACCEL_TAU = 1.5

N = 12
MAX_T = 10.0
T_IDXS_LST = [index_function(idx, max_val=MAX_T, max_idx=N) for idx in range(N+1)]
T_IDXS = np.array(T_IDXS_LST)
FCW_IDXS = T_IDXS < 5.0
T_DIFFS = np.diff(T_IDXS, prepend=[0.])

COMFORT_BRAKE = 2.5
STOP_DISTANCE = 6.0

DISCOMFORT_BRAKE = 5.5
OHSHIT_STOP_DISTANCE = 2.0

# -------------------------------------------------------------------------
# Personality / Following Distance Helpers
# -------------------------------------------------------------------------
def get_jerk_factor(aggressive_jerk_acceleration=0.5, aggressive_jerk_danger=0.5, aggressive_jerk_speed=0.5,
                    standard_jerk_acceleration=1.0, standard_jerk_danger=1.0, standard_jerk_speed=1.0,
                    relaxed_jerk_acceleration=1.0, relaxed_jerk_danger=1.0, relaxed_jerk_speed=1.0,
                    custom_personalities=False, personality=log.LongitudinalPersonality.standard):
  """
  Backward-compatible function signature for jerk factor by personality.
  """
  if custom_personalities:
    if personality == log.LongitudinalPersonality.relaxed:
      return relaxed_jerk_acceleration, relaxed_jerk_danger, relaxed_jerk_speed
    elif personality == log.LongitudinalPersonality.standard:
      return standard_jerk_acceleration, standard_jerk_danger, standard_jerk_speed
    elif personality == log.LongitudinalPersonality.aggressive:
      return aggressive_jerk_acceleration, aggressive_jerk_danger, aggressive_jerk_speed
    else:
      raise NotImplementedError("Longitudinal personality not supported")
  else:
    # Original default: same for all except aggressive gets half jerk cost
    if personality == log.LongitudinalPersonality.relaxed:
      return 1.0, 1.0, 1.0
    elif personality == log.LongitudinalPersonality.standard:
      return 1.0, 1.0, 1.0
    elif personality == log.LongitudinalPersonality.aggressive:
      return 0.5, 0.5, 0.5
    else:
      raise NotImplementedError("Longitudinal personality not supported")

def get_T_FOLLOW(aggressive_follow=1.25,
                 standard_follow=1.45,
                 relaxed_follow=1.75,
                 custom_personalities=False, personality=log.LongitudinalPersonality.standard):
  """
  Backward-compatible function signature for time gap by personality.
  """
  if custom_personalities:
    if personality == log.LongitudinalPersonality.relaxed:
      return relaxed_follow
    elif personality == log.LongitudinalPersonality.standard:
      return standard_follow
    elif personality == log.LongitudinalPersonality.aggressive:
      return aggressive_follow
    else:
      raise NotImplementedError("Longitudinal personality not supported")
  else:
    if personality == log.LongitudinalPersonality.relaxed:
      return 1.75
    elif personality == log.LongitudinalPersonality.standard:
      return 1.45
    elif personality == log.LongitudinalPersonality.aggressive:
      return 1.25
    else:
      raise NotImplementedError("Longitudinal personality not supported")

def get_stopped_equivalence_factor(v_lead):
  return (v_lead**2) / (2 * COMFORT_BRAKE)

def get_safe_obstacle_distance(v_ego, t_follow):
  return (v_ego**2) / (2 * COMFORT_BRAKE) + t_follow * v_ego + STOP_DISTANCE

def desired_follow_distance(v_ego, v_lead, t_follow=None):
  if t_follow is None:
    t_follow = get_T_FOLLOW()
  return get_safe_obstacle_distance(v_ego, t_follow) - get_stopped_equivalence_factor(v_lead)

def get_ohshit_equivalence_factor(v_lead):
  return (v_lead**2) / (2 * DISCOMFORT_BRAKE)

def get_unsafe_obstacle_distance(v_ego, t_follow):
  return (v_ego**2) / (2 * DISCOMFORT_BRAKE) + t_follow * v_ego + OHSHIT_STOP_DISTANCE

def fuck_this_follow_distance(v_ego, v_lead, t_follow=None):
  """
  Original function name unchanged for backward compatibility.
  """
  if t_follow is None:
    t_follow = get_T_FOLLOW()
  return get_unsafe_obstacle_distance(v_ego, t_follow) - get_ohshit_equivalence_factor(v_lead)

# -------------------------------------------------------------------------
# Dynamic Cost Modifiers
# -------------------------------------------------------------------------
# PATCHED: Modified dynamic jerk cost to use a continuous deadzone and smoother transition.
def get_smooth_dynamic_j_ego_cost_array(
    x_obstacle, x_ego, v_ego, v_lead,
    t_follow,
    base_jerk_cost=10.0,       # Reduced base cost from 12.0 to 10.0
    low_jerk_cost=0.5,         # Increased low cost from 0.3 to 0.5
    deadzone_ratio=0.5,        # Wider dead zone
    logistic_k_dist=4.0,       # Softer distance error transition
    logistic_k_speed=1.0,      # Softer speed activation
    min_speed_for_closing=0.3,
    distance_smoothing=5.0,
    time_taper=True,
    speed_factor=1.0,
    danger_response_range=(3.0, 6.0),
    danger_jerk_boost=3.0,
    decel_anticipation_factor=0.35
):
  """
  Returns an array of jerk costs that typically increase when we are too close or
  rapidly closing on the lead. Uses a continuous deadzone and smoother transition.
  """
  if not isinstance(x_obstacle, np.ndarray):
    x_obstacle = np.array([float(x_obstacle)])
  n_steps = x_obstacle.shape[0]

  def ensure_array(val):
    if not isinstance(val, np.ndarray):
      return np.full(n_steps, float(val))
    return val

  x_ego_arr = ensure_array(x_ego)
  v_ego_arr = ensure_array(v_ego)
  v_lead_arr = ensure_array(v_lead)
  t_follow_arr = ensure_array(t_follow)

  desired_dist = desired_follow_distance(v_ego_arr, v_lead_arr, t_follow_arr)
  current_dist = (x_obstacle - x_ego_arr)

  # Continuous dead zone logic for distance error
  deadzone_threshold = deadzone_ratio * desired_dist
  dist_error = desired_dist - current_dist
  delta = np.maximum(np.abs(dist_error) - deadzone_threshold, 0.0)
  dist_activation = 2.0 * (1.0/(1.0 + np.exp(-logistic_k_dist * delta)) - 0.5)

  closing_speed = v_ego_arr - v_lead_arr
  a_ego_arr = np.gradient(v_ego_arr, T_IDXS[:n_steps])
  anticipated_closing = closing_speed + (a_ego_arr * decel_anticipation_factor * T_IDXS[:n_steps])

  danger_factor = np.interp(anticipated_closing, danger_response_range, [1.0, danger_jerk_boost])
  danger_factor = np.clip(danger_factor, 1.0, danger_jerk_boost)
  prev_df = 1.0
  for i in range(n_steps):
    if i > 0 and danger_factor[i] < prev_df:
      danger_factor[i] = max(danger_factor[i], prev_df - 0.5 / T_DIFFS[i])
    prev_df = danger_factor[i]

  closing_shifted = closing_speed - min_speed_for_closing
  speed_logistic = 1.0 / (1.0 + np.exp(-logistic_k_speed * closing_shifted))

  combined_factor = dist_activation * speed_logistic
  jerk_cost_array = low_jerk_cost + combined_factor * (base_jerk_cost - low_jerk_cost)

  # Extended time taper for smoother initial jerk cost
  if time_taper:
    # New linear taper: from 1.3 at t=0 to 1.0 at t=4.0 seconds
    taper_factors = np.interp(T_IDXS[:n_steps], [0.0, 4.0], [1.3, 1.0])
    jerk_cost_array *= taper_factors

  jerk_cost_array *= speed_factor
  return jerk_cost_array

def soft_approach_distance_factor(
    x_obstacle_i, x_ego_i, v_ego_i, v_lead_i, t_follow_i,
    approach_margin=6.0,
    deadzone_margin=1.5,
    max_approach_mult=1.6,
    logistic_k=0.4,
    time_horizon=4.0,  # Internal scaling factor (seconds), NOT tied to T_IDXS
    closing_speed_weight=0.3
):
    # Sigmoid helper function definition
    def sigmoid(x, k=1.0, x0=0.0):
        return 1.0 / (1.0 + np.exp(-k * (x - x0)))

    desired_dist = desired_follow_distance(v_ego_i, v_lead_i, t_follow_i)
    actual_gap = x_obstacle_i - x_ego_i

    closing_speed = v_ego_i - v_lead_i
    anticipated_gap = actual_gap - closing_speed * time_horizon * closing_speed_weight

    # Safety clamp to prevent negative gaps
    anticipated_gap = np.clip(anticipated_gap, 0.0, None)

    approach_ratio = (anticipated_gap - desired_dist) / (approach_margin + deadzone_margin)
    approach_factor = 1.0 + (max_approach_mult - 1.0) * sigmoid(
        -approach_ratio,
        k=logistic_k,
        x0=deadzone_margin/(approach_margin + deadzone_margin)
    )

    return approach_factor

# PATCHED: Smoothed pull-away acceleration ramp by using a linear ramp (exponent=1.0)
# instead of a squared ramp, and reducing the maximum multiplier.
def dynamic_lead_pullaway_distance_cost(
    x_ego_i, v_ego_i,
    x_lead_i, v_lead_i,
    t_follow_i,
    base_cost=3.0,
    pullaway_dist_margin=7.0,
    pullaway_speed_margin=3.0,
    pullaway_max_factor=1.15,
    exponent=1.0
):
  desired_dist = desired_follow_distance(v_ego_i, v_lead_i, t_follow_i)
  actual_gap = x_lead_i - x_ego_i

  gap_excess = actual_gap - desired_dist
  speed_diff = v_lead_i - v_ego_i

  if gap_excess <= 0 or speed_diff <= 0:
    return base_cost

  # Use a linear ramp (exponent=1.0) for smoother pull-away behavior.
  z_dist = (gap_excess / pullaway_dist_margin) ** exponent
  z_speed = (speed_diff / pullaway_speed_margin) ** exponent
  z = min(z_dist, z_speed)

  factor = 1.0 + (pullaway_max_factor - 1.0) * z
  return base_cost * np.clip(factor, 1.0, pullaway_max_factor)

def dynamic_lead_constraint_weight(
    x_obstacle_i, x_ego_i, v_ego_i, v_lead_i, t_follow_i,
    min_penalty=5.0,
    max_penalty=1e5,
    dist_margin=2.0,
    speed_margin=2.0
):
  """
  Returns a penalty factor for the lead-distance constraint. The penalty
  grows if we are too close to the desired following distance (plus a
  margin) or if we are closing on the lead too quickly. Clamped between
  min_penalty and max_penalty.
  """
  desired_dist = desired_follow_distance(v_ego_i, v_lead_i, t_follow_i)
  actual_gap = x_obstacle_i - x_ego_i
  dist_gap_below = (desired_dist + dist_margin) - actual_gap  # >0 => too close

  speed_diff = v_ego_i - (v_lead_i + speed_margin)           # >0 => closing quickly

  penalty_activation = 0.0
  if dist_gap_below > 0.0:
    penalty_activation += dist_gap_below
  if speed_diff > 0.0:
    # Weight speed closing a bit so it doesn’t dominate
    penalty_activation += speed_diff * 0.5

  # Quadratic scaling on the combined “over-limit” amounts
  penalty_raw = min_penalty + 100.0 * (penalty_activation ** 2)
  return float(np.clip(penalty_raw, min_penalty, max_penalty))

# -------------------------------------------------------------------------
# ACADOS Model + OCP Setup
# -------------------------------------------------------------------------
def gen_long_model():
  from openpilot.third_party.acados.acados_template import AcadosModel

  model = AcadosModel()
  model.name = MODEL_NAME

  x_ego = SX.sym('x_ego')
  v_ego = SX.sym('v_ego')
  a_ego = SX.sym('a_ego')
  model.x = vertcat(x_ego, v_ego, a_ego)

  j_ego = SX.sym('j_ego')
  model.u = vertcat(j_ego)

  x_ego_dot = SX.sym('x_ego_dot')
  v_ego_dot = SX.sym('v_ego_dot')
  a_ego_dot = SX.sym('a_ego_dot')
  model.xdot = vertcat(x_ego_dot, v_ego_dot, a_ego_dot)

  a_min = SX.sym('a_min')
  a_max = SX.sym('a_max')
  x_obstacle = SX.sym('x_obstacle')
  prev_a = SX.sym('prev_a')
  lead_t_follow = SX.sym('lead_t_follow')
  lead_danger_factor = SX.sym('lead_danger_factor')
  model.p = vertcat(a_min, a_max, x_obstacle, prev_a, lead_t_follow, lead_danger_factor)

  f_expl = vertcat(v_ego, a_ego, j_ego)
  model.f_impl_expr = model.xdot - f_expl
  model.f_expl_expr = f_expl

  return model

def gen_long_ocp():
  from openpilot.third_party.acados.acados_template import AcadosOcp

  ocp = AcadosOcp()
  ocp.model = gen_long_model()
  Tf = T_IDXS[-1]
  ocp.dims.N = N

  ocp.cost.cost_type = 'NONLINEAR_LS'
  ocp.cost.cost_type_e = 'NONLINEAR_LS'

  QR = np.zeros((COST_DIM, COST_DIM))
  Q = np.zeros((COST_E_DIM, COST_E_DIM))
  ocp.cost.W = QR
  ocp.cost.W_e = Q

  x_ego, v_ego, a_ego = ocp.model.x[0], ocp.model.x[1], ocp.model.x[2]
  j_ego = ocp.model.u[0]

  a_min, a_max = ocp.model.p[0], ocp.model.p[1]
  x_obstacle = ocp.model.p[2]
  prev_a = ocp.model.p[3]
  lead_t_follow = ocp.model.p[4]
  lead_danger_factor = ocp.model.p[5]

  ocp.cost.yref = np.zeros((COST_DIM, ))
  ocp.cost.yref_e = np.zeros((COST_E_DIM, ))

  # --- TWEAK: Increase alpha from 0.6 to 0.7 for centering in the safe gap ---
  desired_dist_comfort = get_safe_obstacle_distance(v_ego, lead_t_follow)
  small_speed_offset = 5.0
  alpha = 0.7
  raw_gap = ((x_obstacle - x_ego) - desired_dist_comfort)
  dist_err_mixed = alpha * raw_gap + (1.0 - alpha) * (raw_gap / (v_ego + small_speed_offset))

  costs = [
    dist_err_mixed,         # 0
    x_ego,                  # 1
    v_ego,                  # 2
    a_ego,                  # 3
    a_ego - prev_a,         # 4
    j_ego,                  # 5
  ]
  ocp.model.cost_y_expr = vertcat(*costs)
  ocp.model.cost_y_expr_e = vertcat(*costs[:-1])  # omit j_ego for final stage

  # constraints: v_ego >= 0, a_min <= a_ego <= a_max, lead constraint
  constraints = vertcat(
    v_ego,
    (a_ego - a_min),
    (a_max - a_ego),
    (x_obstacle - x_ego) - lead_danger_factor * desired_dist_comfort
  )
  ocp.model.con_h_expr = constraints

  x0 = np.zeros(X_DIM)
  ocp.constraints.x0 = x0
  ocp.parameter_values = np.array([
    -1.2, 1.2, 0.0, 0.0,
    get_T_FOLLOW(),
    LEAD_DANGER_FACTOR
  ])

  cost_weights = np.zeros(CONSTR_DIM)
  ocp.cost.zl = cost_weights
  ocp.cost.Zl = cost_weights
  ocp.cost.Zu = cost_weights
  ocp.cost.zu = cost_weights

  ocp.constraints.lh = np.zeros(CONSTR_DIM)
  ocp.constraints.uh = 1e4 * np.ones(CONSTR_DIM)
  ocp.constraints.idxsh = np.arange(CONSTR_DIM)

  ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
  ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
  ocp.solver_options.integrator_type = 'ERK'
  ocp.solver_options.nlp_solver_type = ACADOS_SOLVER_TYPE
  ocp.solver_options.qp_solver_cond_N = 1
  ocp.solver_options.qp_solver_iter_max = 10
  ocp.solver_options.qp_tol = 1e-3
  ocp.solver_options.tf = Tf
  ocp.solver_options.shooting_nodes = T_IDXS

  ocp.code_export_directory = EXPORT_DIR
  return ocp

# -------------------------------------------------------------------------
# The MPC Class
# -------------------------------------------------------------------------
class LongitudinalMpc:
  def __init__(self, mode='acc', dt=DT_MDL):
    self._approach_factor_last = 1.0  # For smoothing dynamic multiplier changes
    self.mode = mode
    self.dt = dt

    self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.reset()
    self.source = SOURCES[2]

  def reset(self):
    self.solver.reset()
    self.v_solution = np.zeros(N+1)
    self.a_solution = np.zeros(N+1)
    self.prev_a = np.array(self.a_solution)
    self.j_solution = np.zeros(N)
    self.yref = np.zeros((N+1, COST_DIM))

    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
    self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])

    self.x_sol = np.zeros((N+1, X_DIM))
    self.u_sol = np.zeros((N, 1))
    self.params = np.zeros((N+1, PARAM_DIM))

    for i in range(N+1):
      self.solver.set(i, 'x', np.zeros(X_DIM))

    self.last_cloudlog_t = 0
    self.status = False
    self.crash_cnt = 0.0
    self.solution_status = 0
    self.solve_time = 0.0
    self.time_qp_solution = 0.0
    self.time_linearization = 0.0
    self.time_integrator = 0.0

    self.x0 = np.zeros(X_DIM)

    # Default cost multipliers
    self.acceleration_jerk_factor = 1.0
    self.danger_jerk_factor = 1.0
    self.speed_jerk_factor = 1.0

    self.set_weights()

  def set_cost_weights(self, cost_weights, constraint_cost_weights):
    # Single place to set the costs for each step
    W = np.asfortranarray(np.diag(cost_weights))
    for i in range(N):
      # Instead of going to 0 after 2s, keep at 30% to reduce abrupt flips
      W[4,4] = cost_weights[4] * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.3])
      self.solver.cost_set(i, 'W', W)
    self.solver.cost_set(N, 'W', np.copy(W[:COST_E_DIM, :COST_E_DIM]))

    Zl = np.array(constraint_cost_weights)
    for i in range(N):
      self.solver.cost_set(i, 'Zl', Zl)

  def set_weights(self, acceleration_jerk=1.0, danger_jerk=1.0, speed_jerk=1.0,
                  prev_accel_constraint=True, personality=log.LongitudinalPersonality.standard,
                  speed_scaling=1.0):
    """
    Backward-compatible method for setting cost weights from external code.
    """
    self.acceleration_jerk_factor = acceleration_jerk
    self.danger_jerk_factor = danger_jerk
    self.speed_jerk_factor = speed_jerk

    if self.mode == 'acc':
      a_change_cost = A_CHANGE_COST if prev_accel_constraint else 0.0
      j_cost = J_EGO_COST * self.acceleration_jerk_factor

      self.base_cost_weights = [
        X_EGO_OBSTACLE_COST,
        X_EGO_COST,
        V_EGO_COST,
        A_EGO_COST,
        a_change_cost,
        j_cost
      ]
      # The last constraint cost is replaced dynamically, but we keep a
      # placeholder for initialization
      self.base_constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, danger_jerk]

      self.set_cost_weights(self.base_cost_weights, self.base_constraint_cost_weights)

    elif self.mode == 'blended':
      a_change_cost = 40.0 if prev_accel_constraint else 0.0
      j_cost = 1.0 * self.acceleration_jerk_factor
      cost_weights = [0., 0.1, 0.2, 5.0, a_change_cost, j_cost]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, 50.0]
      self.base_cost_weights = cost_weights
      self.base_constraint_cost_weights = constraint_cost_weights
      self.set_cost_weights(cost_weights, constraint_cost_weights)

    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized')

  def set_cur_state(self, v, a):
    v_prev = self.x0[1]
    self.x0[1] = v
    self.x0[2] = a
    if abs(v_prev - v) > 2.:
      for i in range(N+1):
        self.solver.set(i, 'x', self.x0)

  @staticmethod
  def extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau):
    a_lead_traj = a_lead * np.exp(-a_lead_tau * (T_IDXS**2)/2.)
    v_lead_traj = np.clip(v_lead + np.cumsum(T_DIFFS * a_lead_traj), 0.0, 1e8)
    x_lead_traj = x_lead + np.cumsum(T_DIFFS * v_lead_traj)
    return np.column_stack((x_lead_traj, v_lead_traj))

  def process_lead(self, lead):
    v_ego = self.x0[1]
    if lead is not None and lead.status:
      x_lead = lead.dRel
      v_lead = lead.vLead
      a_lead = lead.aLeadK
      a_lead_tau = lead.aLeadTau
    else:
      x_lead = 50.0
      v_lead = v_ego + 10.0
      a_lead = 0.0
      a_lead_tau = LEAD_ACCEL_TAU

    min_x_lead = ((v_ego + v_lead)/2) * (v_ego - v_lead) / (-ACCEL_MIN * 2)
    x_lead = clip(x_lead, min_x_lead, 1e8)
    v_lead = clip(v_lead, 0.0, 1e8)
    a_lead = clip(a_lead, -10., 5.)

    return self.extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau)

  def set_accel_limits(self, min_a, max_a):
    self.cruise_min_a = min_a
    self.max_a = max_a

  def _apply_dynamic_costs(self, chosen_lead_xv):
    """
    Unified dynamic cost logic, referencing the previous iteration's solution
    so we don't chase ourselves mid-iteration.
    PATCHED: Updated parameters to use smoother jerk cost behavior and a smoother pull-away ramp.
    """
    base_j_cost = 12.0 * (self.acceleration_jerk_factor or 1.0)
    # Scale base_j_cost to match new default (10.0 instead of 12.0)
    scaled_base_j_cost = base_j_cost * (10.0/12.0)

    dyn_j_ego_array = get_smooth_dynamic_j_ego_cost_array(
      x_obstacle=self.params[:,2],
      x_ego=self.x_sol[:,0],
      v_ego=self.x_sol[:,1],
      v_lead=chosen_lead_xv[:,1] if chosen_lead_xv.shape[1] > 1 else self.x_sol[:,1],
      t_follow=self.params[:,4],
      base_jerk_cost=scaled_base_j_cost,
      low_jerk_cost=0.5,
      deadzone_ratio=0.5,
      logistic_k_dist=4.0,
      logistic_k_speed=1.0,
      distance_smoothing=4.0,
      time_taper=True,
      speed_factor=1.0,
      danger_response_range=(3.0, 6.0),
      danger_jerk_boost=3.0,
      decel_anticipation_factor=0.35
    )

    dist_cost_base = self.base_cost_weights[0]
    a_change_cost = self.base_cost_weights[4]

    for i in range(N):
      # Start from the existing base cost matrix
      W_step = np.diag(self.base_cost_weights)
      # Keep partial a_change cost after 2s (30%)
      W_step[4,4] = a_change_cost * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.3])
      # Apply dynamic jerk cost
      W_step[5,5] = dyn_j_ego_array[i]

      x_ego_i, v_ego_i_iter, _ = self.x_sol[i]
      if i < chosen_lead_xv.shape[0]:
        x_lead_i, v_lead_i = chosen_lead_xv[i]
      else:
        x_lead_i, v_lead_i = chosen_lead_xv[-1]

      t_follow_i = self.params[i,4]

      # PATCHED: Use a smoother pull-away ramp with a linear exponent and reduced maximum factor.
      pullaway_dist_cost = dynamic_lead_pullaway_distance_cost(
          x_ego_i, v_ego_i_iter,
          x_lead_i, v_lead_i,
          t_follow_i,
          base_cost=dist_cost_base,
          pullaway_max_factor=1.15,
          exponent=1.0
      )

      approach_factor = soft_approach_distance_factor(
          x_lead_i, x_ego_i, v_ego_i_iter, v_lead_i, t_follow_i,
          approach_margin=5.0,
          max_approach_mult=1.5,
          logistic_k=1.0
      )

      combined_factor = pullaway_dist_cost * approach_factor
      clamped_factor = np.clip(combined_factor, 1.0, 1.3)

      # Increase smoothing on the dynamic multiplier to further slow transitions
      smoothed_factor = 0.9 * self._approach_factor_last + 0.1 * clamped_factor
      self._approach_factor_last = smoothed_factor
      W_step[0,0] = dist_cost_base * smoothed_factor

      # This call now succeeds because the function is defined above
      last_constraint_penalty = dynamic_lead_constraint_weight(
          x_obstacle_i=self.params[i,2],
          x_ego_i=x_ego_i,
          v_ego_i=v_ego_i_iter,
          v_lead_i=v_lead_i,
          t_follow_i=t_follow_i,
          min_penalty=5.0,
          max_penalty=1e5,
          dist_margin=2.0,
          speed_margin=2.0
      )

      Zl = np.array([LIMIT_COST, LIMIT_COST, LIMIT_COST, last_constraint_penalty])
      self.solver.cost_set(i, 'Zl', Zl)
      self.solver.cost_set(i, 'W', W_step)

    # Terminal stage
    W_e = np.copy(np.diag(self.base_cost_weights[:COST_E_DIM]))
    self.solver.cost_set(N, 'W', W_e)

  def update(self, lead_one, lead_two, v_cruise, x, v, a, j, radarless_model, t_follow,
             trafficModeActive, personality=log.LongitudinalPersonality.standard):
    v_ego = self.x0[1]
    self.status = lead_one.status or lead_two.status

    # Process leads
    lead_xv_0 = self.process_lead(lead_one)
    lead_xv_1 = self.process_lead(lead_two)

    lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor(lead_xv_0[:,1])
    lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor(lead_xv_1[:,1])

    # Set final min_a / max_a
    self.params[:,0] = self.cruise_min_a
    self.params[:,1] = self.max_a

    if self.mode == 'acc':
      self.params[:,5] = LEAD_DANGER_FACTOR

      v_lower = v_ego + (T_IDXS * self.cruise_min_a * 1.05)
      v_upper = v_ego + (T_IDXS * self.max_a * 1.05)
      v_cruise_clipped = np.clip(v_cruise * np.ones(N+1), v_lower, v_upper)

      cruise_obstacle = (
        np.cumsum(T_DIFFS * v_cruise_clipped)
        + get_safe_obstacle_distance(v_cruise_clipped, t_follow)
      )

      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])
      self.source = SOURCES[np.argmin(x_obstacles[0])]

      # In ACC mode, we primarily handle leads + cruise
      x[:] = 0.0
      v[:] = 0.0
      a[:] = 0.0
      j[:] = 0.0

    elif self.mode == 'blended':
      self.params[:,5] = 1.0
      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle])

      # Limit x with cruise
      cruise_target = T_IDXS * np.clip(v_cruise, v_ego - 2.0, 1e3) + x[0]
      xforward = ((v[1:] + v[:-1]) / 2) * (T_IDXS[1:] - T_IDXS[:-1])
      x = np.cumsum(np.insert(xforward, 0, x[0]))
      x_and_cruise = np.column_stack([x, cruise_target])
      x = np.min(x_and_cruise, axis=1)

      self.source = 'e2e' if x_and_cruise[1,0] < x_and_cruise[1,1] else 'cruise'

    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized')

    # Fill cost references
    self.yref[:,1] = x
    self.yref[:,2] = v
    self.yref[:,3] = a
    self.yref[:,5] = j
    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
    self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])

    # Pick min obstacle
    self.params[:,2] = np.min(x_obstacles, axis=1)
    self.params[:,3] = np.copy(self.prev_a)
    self.params[:,4] = t_follow

    # Choose a single lead's extrapolation for cost shaping
    chosen_lead_xv = lead_xv_0

    # Dynamic cost logic
    self._apply_dynamic_costs(chosen_lead_xv)

    # Solve
    self.run()

    # FCW check
    lead_probability = lead_one.prob if radarless_model else lead_one.modelProb
    if (np.any(lead_xv_0[FCW_IDXS,0] - self.x_sol[FCW_IDXS,0] < CRASH_DISTANCE) and lead_probability > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

    if self.mode == 'blended':
      if any((lead_0_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], t_follow)) - self.x_sol[:,0] < 0.0):
        self.source = 'lead0'
      if any((lead_1_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], t_follow)) - self.x_sol[:,0] < 0.0) and \
         (lead_1_obstacle[0] - lead_0_obstacle[0]):
        self.source = 'lead1'

  def run(self):
    # Set solver parameters
    for i in range(N+1):
      self.solver.set(i, 'p', self.params[i])

    # initial condition
    self.solver.constraints_set(0, "lbx", self.x0)
    self.solver.constraints_set(0, "ubx", self.x0)

    self.solution_status = self.solver.solve()
    self.solve_time = float(self.solver.get_stats('time_tot')[0])
    self.time_qp_solution = float(self.solver.get_stats('time_qp')[0])
    self.time_linearization = float(self.solver.get_stats('time_lin')[0])
    self.time_integrator = float(self.solver.get_stats('time_sim')[0])

    for i in range(N+1):
      self.x_sol[i] = self.solver.get(i, 'x')
    for i in range(N):
      self.u_sol[i] = self.solver.get(i, 'u')

    self.v_solution = self.x_sol[:,1]
    self.a_solution = self.x_sol[:,2]
    self.j_solution = self.u_sol[:,0]

    self.prev_a = np.interp(T_IDXS + self.dt, T_IDXS, self.a_solution)

    t = time.monotonic()
    if self.solution_status != 0:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning(f"Long MPC reset, solution_status: {self.solution_status}")
      self.reset()

if __name__ == "__main__":
  from openpilot.third_party.acados.acados_template import AcadosOcp, AcadosOcpSolver
  ocp = gen_long_ocp()
  AcadosOcpSolver.generate(ocp, json_file=JSON_FILE)