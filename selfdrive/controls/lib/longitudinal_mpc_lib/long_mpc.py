#!/usr/bin/env python3
import os
import time
import math
import numpy as np
from cereal import log
from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
# WARNING: imports outside of constants will not trigger a rebuild
from openpilot.selfdrive.modeld.constants import index_function
from openpilot.selfdrive.car.interfaces import ACCEL_MIN

if __name__ == '__main__':  # generating code
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

X_EGO_OBSTACLE_COST = 3.
X_EGO_COST = 0.
V_EGO_COST = 0.
A_EGO_COST = 0.
J_EGO_COST = 5.0
A_CHANGE_COST = 200.
DANGER_ZONE_COST = 100.
CRASH_DISTANCE = .25
LEAD_DANGER_FACTOR = 0.75
LIMIT_COST = 1e6
ACADOS_SOLVER_TYPE = 'SQP_RTI'
# Default lead acceleration decay set to 50% at 1s
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


# --------------------------------------------------------------------------------------
# Tuning / Follow-Distance Helpers
# --------------------------------------------------------------------------------------
def get_jerk_factor(aggressive_jerk_acceleration=0.5, aggressive_jerk_danger=0.5, aggressive_jerk_speed=0.5,
                    standard_jerk_acceleration=1.0, standard_jerk_danger=1.0, standard_jerk_speed=1.0,
                    relaxed_jerk_acceleration=1.0, relaxed_jerk_danger=1.0, relaxed_jerk_speed=1.0,
                    custom_personalities=False, personality=log.LongitudinalPersonality.standard):
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
    if personality == log.LongitudinalPersonality.relaxed:
      return 1.0, 1.0, 1.0
    elif personality == log.LongitudinalPersonality.standard:
      return 1.0, 1.0, 1.0
    elif personality == log.LongitudinalPersonality.aggressive:
      return 0.5, 0.5, 0.5
    else:
      raise NotImplementedError("Longitudinal personality not supported")


def get_T_FOLLOW(aggressive_follow=1.25, standard_follow=1.45, relaxed_follow=1.75,
                 custom_personalities=False, personality=log.LongitudinalPersonality.standard):
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
  if t_follow is None:
    t_follow = get_T_FOLLOW()
  return get_unsafe_obstacle_distance(v_ego, t_follow) - get_ohshit_equivalence_factor(v_lead)


# --------------------------------------------------------------------------------------
# Dynamic Jerk Cost
# --------------------------------------------------------------------------------------
def get_smooth_dynamic_j_ego_cost_array(
    x_obstacle, x_ego, v_ego, v_lead,
    t_follow, base_jerk_cost=5.0,
    low_jerk_cost=0.5,
    logistic_k_dist=8.0,
    logistic_k_speed=3.0,
    min_speed_for_closing=0.1,
    distance_smoothing=5.0,
    time_taper=False,
    speed_factor=1.0,
):
  """
  Smoothly transitions jerk cost between "limo" (when close & closing) 
  and "Andretti" (when comfortable).
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
  dist_ratio = (desired_dist - current_dist) / (desired_dist + distance_smoothing)

  def logistic(z):
    return 1.0 / (1.0 + np.exp(-z))

  dist_logistic = logistic(logistic_k_dist * dist_ratio)
  closing_speed = v_ego_arr - v_lead_arr
  closing_shifted = closing_speed - min_speed_for_closing
  speed_logistic = logistic(logistic_k_speed * closing_shifted)
  combined_factor = dist_logistic * speed_logistic

  jerk_cost_array = low_jerk_cost + (1.0 - combined_factor) * (base_jerk_cost - low_jerk_cost)

  if time_taper:
    taper_factors = []
    for t in T_IDXS[:n_steps]:
      if t <= 2.0:
        alpha = t / 2.0
        factor_t = 1.5 + (1.0 - 1.5) * alpha
      else:
        factor_t = 1.0
      taper_factors.append(factor_t)
    taper_factors = np.array(taper_factors)
    jerk_cost_array *= taper_factors

  jerk_cost_array *= speed_factor
  return jerk_cost_array


# --------------------------------------------------------------------------------------
# ACADOS Model Definition
# --------------------------------------------------------------------------------------
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

  desired_dist_comfort = get_safe_obstacle_distance(v_ego, lead_t_follow)

  costs = [
    ((x_obstacle - x_ego) - desired_dist_comfort) / (v_ego + 10.),  # distance error
    x_ego,          # cost=0 by default
    v_ego,          # cost=0 by default
    a_ego,          # cost=0 by default
    a_ego - prev_a, # cost on accel change
    j_ego,          # jerk cost
  ]
  ocp.model.cost_y_expr = vertcat(*costs)
  ocp.model.cost_y_expr_e = vertcat(*costs[:-1])

  constraints = vertcat(
    v_ego,
    (a_ego - a_min),
    (a_max - a_ego),
    ((x_obstacle - x_ego) - lead_danger_factor * (desired_dist_comfort)) / (v_ego + 10.)
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
  ocp.constraints.uh = 1e4*np.ones(CONSTR_DIM)
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


# --------------------------------------------------------------------------------------
# Dynamic weighting of 4th constraint for near lead (like before),
# plus a new "pull-away boost" for distance-error cost if the lead is pulling away.
# --------------------------------------------------------------------------------------

def dynamic_lead_constraint_weight(x_obstacle_i, x_ego_i, v_ego_i, v_lead_i,
                                   t_follow_i,
                                   min_penalty=10.0,
                                   max_penalty=1e6,
                                   dist_margin=2.0,
                                   speed_margin=2.0):
  """
  Returns a dynamic penalty for the lead-distance constraint (#4).
  This makes the solver extremely unwilling to get too close (and very quickly).
  """
  desired_dist = get_safe_obstacle_distance(v_ego_i, t_follow_i)
  actual_gap = x_obstacle_i - x_ego_i
  gap_error = desired_dist - actual_gap
  closing_speed = v_ego_i - v_lead_i

  # We'll do a simple approach: the bigger (gap_error + closing_speed) is, 
  # the closer we are to a problem.  
  z = (gap_error / dist_margin) + (closing_speed / speed_margin)
  # Clip so negative => no penalty inflation
  z_clipped = max(z, 0.0)

  # linear ramp from min_penalty->max_penalty
  alpha = min(z_clipped, 1.0)  # you could do logistic, but linear is simpler
  penalty = min_penalty + alpha * (max_penalty - min_penalty)
  return penalty


def dynamic_lead_pullaway_distance_cost(x_ego_i, v_ego_i, x_lead_i, v_lead_i,
                                        t_follow_i,
                                        base_cost=3.0,
                                        pullaway_dist_margin=5.0,
                                        pullaway_speed_margin=2.0,
                                        pullaway_max_factor=4.0):
  """
  Returns a factor for the "distance error" cost (#0 in the cost vector) 
  that gets larger if the lead is pulling away and ego is lagging behind.

  This encourages the solver to accelerate more strongly, 
  but remains mild if the lead is not truly "walking away."
  """
  desired_dist = desired_follow_distance(v_ego_i, v_lead_i, t_follow_i)
  actual_gap = x_lead_i - x_ego_i
  # Gap excess: how much bigger is the gap than the desired
  gap_excess = actual_gap - desired_dist

  # Speed diff: lead minus ego
  speed_diff = v_lead_i - v_ego_i

  # If both gap_excess > 0 and speed_diff > 0, lead is pulling away
  # We'll define a small measure z to detect "pull away intensity."
  # For example, if gap_excess = 5m and speed_diff = 2m/s, 
  # then we want to ramp up the cost to catch up. 
  # We'll do a simple linear approach:
  z_dist = gap_excess / pullaway_dist_margin
  z_speed = speed_diff / pullaway_speed_margin
  # We'll only care if both are > 0
  z = min(z_dist, z_speed)
  if z < 0.0:
    # Not pulling away
    return base_cost

  # Clip or scale up to a max factor
  # e.g. if z=1 => factor ~ pullaway_max_factor 
  # if z=0 => factor=1 
  # We do factor= 1 + z*(pullaway_max_factor-1) 
  # then clip at pullaway_max_factor
  factor = 1.0 + z*(pullaway_max_factor - 1.0)
  factor = np.clip(factor, 1.0, pullaway_max_factor)
  return base_cost * factor


class LongitudinalMpc:
  def __init__(self, mode='acc', dt=DT_MDL):
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
    self.u_sol = np.zeros((N,1))
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

    # We'll store a "base" set for the cost, then re-apply time-varying changes in update()
    self.set_weights()

  def set_cost_weights(self, cost_weights, constraint_cost_weights):
    """
    Original function for 'blended' or legacy usage.
    """
    W = np.asfortranarray(np.diag(cost_weights))
    for i in range(N):
      W[4,4] = cost_weights[4] * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.0])
      self.solver.cost_set(i, 'W', W)
    self.solver.cost_set(N, 'W', np.copy(W[:COST_E_DIM, :COST_E_DIM]))

    Zl = np.array(constraint_cost_weights)
    for i in range(N):
      self.solver.cost_set(i, 'Zl', Zl)

  def set_weights(self, acceleration_jerk=1.0, danger_jerk=1.0, speed_jerk=1.0,
                  prev_accel_constraint=True, personality=log.LongitudinalPersonality.standard,
                  speed_scaling=1.0):
    """
    Default ACC mode sets *base* weights, but doesn't finalize them until we do
    our time-varying updates in 'update_timevarying_costs()'.
    """
    if self.mode == 'acc':
      # We'll just store minimal placeholders here:
      a_change_cost = acceleration_jerk if prev_accel_constraint else 0.0
      # The base cost array: (distance error, x, v, a, (a-prev_a), jerk)
      self.base_cost_weights = [3.0, 0.0, 0.0, 0.0, a_change_cost, 1.0]

      # For constraints #1,#2,#3 => large penalty, #4 => placeholder
      self.base_constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, 50.0]

      # We'll set them, but in .update() we re-apply dynamic changes
      W = np.diag(self.base_cost_weights)
      for i in range(N):
        # reduce cost on (a-a_prev) over time
        W[4,4] = a_change_cost * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.0])
        self.solver.cost_set(i, 'W', W)
      W_e = np.copy(W[:COST_E_DIM, :COST_E_DIM])
      self.solver.cost_set(N, 'W', W_e)

      Zl = np.array(self.base_constraint_cost_weights)
      for i in range(N):
        self.solver.cost_set(i, 'Zl', Zl)

    elif self.mode == 'blended':
      # Legacy path
      a_change_cost = 40.0 if prev_accel_constraint else 0
      cost_weights = [0., 0.1, 0.2, 5.0, a_change_cost, 1.0]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, 50.0]
      self.set_cost_weights(cost_weights, constraint_cost_weights)
    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner cost set')

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

  def update_timevarying_costs(self, lead_xv):
    """
    Re-apply dynamic cost weighting for each time step:
      1) Raise distance-error cost if the lead is pulling away. 
      2) Apply dynamic jerk cost as before.
      3) Keep constraints #1..#3 at large penalty, #4 dynamic.
    """
    # Precompute the dynamic jerk cost:
    dyn_j_ego_array = get_smooth_dynamic_j_ego_cost_array(
      x_obstacle=self.params[:,2],
      x_ego=self.x0[0],
      v_ego=self.x0[1],
      v_lead=self.x0[1],
      t_follow=self.params[:,4],
      base_jerk_cost=5.0,
      low_jerk_cost=0.5,
      logistic_k_dist=8.0,
      logistic_k_speed=3.0,
      min_speed_for_closing=0.3,
      distance_smoothing=5.0,
      time_taper=True,
      speed_factor=1.0,
    )

    for i in range(N):
      # Rebuild the base cost matrix from self.base_cost_weights
      W_step = np.diag(self.base_cost_weights)

      # (a_ego - prev_a) cost decays over time
      a_change_cost = self.base_cost_weights[4]
      W_step[4,4] = a_change_cost * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.0])

      # Jerk cost is dynamic
      W_step[5,5] = dyn_j_ego_array[i]

      # Now apply the “pull-away boost” to the distance-error cost (index 0)
      # We'll retrieve x_ego, v_ego from the solver state for more accuracy:
      x_sol_i = self.solver.get(i, 'x')
      x_ego_i, v_ego_i = x_sol_i[0], x_sol_i[1]
      # For the lead, we might use the chosen lead's data at horizon i
      if i < lead_xv.shape[0]:
        x_lead_i, v_lead_i = lead_xv[i]
      else:
        x_lead_i, v_lead_i = lead_xv[-1]

      t_follow_i = self.params[i,4] if i < (N+1) else get_T_FOLLOW()
      dist_cost_base = self.base_cost_weights[0]

      new_dist_cost = dynamic_lead_pullaway_distance_cost(
        x_ego_i, v_ego_i,
        x_lead_i, v_lead_i,
        t_follow_i,
        base_cost=dist_cost_base,
        pullaway_dist_margin=5.0,
        pullaway_speed_margin=2.0,
        pullaway_max_factor=4.0
      )
      W_step[0,0] = new_dist_cost

      self.solver.cost_set(i, 'W', W_step)

      # Re-set the constraint weighting (#4) dynamically so it gets huge if we’re too close
      v_ego_state = v_ego_i
      x_ego_state = x_ego_i
      last_constraint_penalty = dynamic_lead_constraint_weight(
        x_obstacle_i=self.params[i,2],
        x_ego_i=x_ego_state,
        v_ego_i=v_ego_state,
        v_lead_i=v_ego_state,  # if you track actual lead speed, pass it
        t_follow_i=t_follow_i,
        min_penalty=10.0,
        max_penalty=1e6,
        dist_margin=2.0,
        speed_margin=2.0
      )
      # Keep constraints #1..#3 at LIMIT_COST, #4 dynamic
      Zl = np.array([LIMIT_COST, LIMIT_COST, LIMIT_COST, last_constraint_penalty])
      self.solver.cost_set(i, 'Zl', Zl)

    # For the final stage, ignore jerk cost but keep distance, etc.
    W_e = np.copy(np.diag(self.base_cost_weights[:COST_E_DIM]))
    self.solver.cost_set(N, 'W', W_e)


  def update(self, lead_one, lead_two, v_cruise, x, v, a, j, radarless_model, t_follow,
             trafficModeActive, personality=log.LongitudinalPersonality.standard):
    v_ego = self.x0[1]
    self.status = lead_one.status or lead_two.status

    lead_xv_0 = self.process_lead(lead_one)
    lead_xv_1 = self.process_lead(lead_two)

    lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor(lead_xv_0[:,1])
    lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor(lead_xv_1[:,1])

    self.params[:,0] = ACCEL_MIN
    self.params[:,1] = max(0.0, self.max_a)

    if self.mode == 'acc':
      self.params[:,5] = LEAD_DANGER_FACTOR

      v_lower = v_ego + (T_IDXS * self.cruise_min_a * 1.05)
      v_upper = v_ego + (T_IDXS * self.max_a * 1.05)
      v_cruise_clipped = np.clip(v_cruise * np.ones(N+1), v_lower, v_upper)
      cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, t_follow)

      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])
      self.source = SOURCES[np.argmin(x_obstacles[0])]

      x[:], v[:], a[:], j[:] = 0.0, 0.0, 0.0, 0.0

    elif self.mode == 'blended':
      self.params[:,5] = 1.0
      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle])
      cruise_target = T_IDXS * np.clip(v_cruise, v_ego - 2.0, 1e3) + x[0]
      xforward = ((v[1:] + v[:-1]) / 2) * (T_IDXS[1:] - T_IDXS[:-1])
      x = np.cumsum(np.insert(xforward, 0, x[0]))
      x_and_cruise = np.column_stack([x, cruise_target])
      x = np.min(x_and_cruise, axis=1)
      self.source = 'e2e' if x_and_cruise[1,0] < x_and_cruise[1,1] else 'cruise'
    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized')

    self.yref[:,1] = x
    self.yref[:,2] = v
    self.yref[:,3] = a
    self.yref[:,5] = j
    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
    self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])

    self.params[:,2] = np.min(x_obstacles, axis=1)
    self.params[:,3] = np.copy(self.prev_a)
    self.params[:,4] = t_follow

    # Here: apply dynamic “pull-away” cost changes & near-lead constraints
    # We'll pick lead_xv_0 for reference, or whichever is “closest”
    # For simplicity, assume lead0 is the main lead:
    chosen_lead_xv = lead_xv_0
    self.update_timevarying_costs(chosen_lead_xv)

    self.run()

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
    for i in range(N+1):
      self.solver.set(i, 'p', self.params[i])
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
        cloudlog.warning(f"Long mpc reset, solution_status: {self.solution_status}")
      self.reset()


if __name__ == "__main__":
  from openpilot.third_party.acados.acados_template import AcadosOcp, AcadosOcpSolver
  ocp = gen_long_ocp()
  AcadosOcpSolver.generate(ocp, json_file=JSON_FILE)
  # AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)