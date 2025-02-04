#!/usr/bin/env python3
"""
Revised MPC script (backward compatible) with dynamic tuning adjustments.

Changes from the previous version:
  • The follow behavior remains as before.
  • In dynamic_lead_pullaway_distance_cost(), if the lead is decelerating (speed_diff <= 0)
    then a higher cost multiplier (e.g. 1.5×) is returned so that the MPC is compelled
    to decelerate more decisively.
  • All imports and module names are kept exactly as in the upstream version.
  
This should make the controller very smooth in steady follow yet “snappy” (and safe)
when the lead slows or stops.
"""

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

# -----------------------------------------------------------------------------
# Constants and Configuration
# -----------------------------------------------------------------------------
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
J_EGO_COST = 5.0       # Base jerk cost
A_CHANGE_COST = 150.0

CRASH_DISTANCE = 0.25
LEAD_DANGER_FACTOR = 0.75
DANGER_ZONE_COST = 100.0  # Reintroduced for backward compatibility
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

# -----------------------------------------------------------------------------
# Personality / Following Distance Helpers
# -----------------------------------------------------------------------------
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

# -----------------------------------------------------------------------------
# Dynamic Cost Modifiers
# -----------------------------------------------------------------------------
def get_smooth_dynamic_j_ego_cost_array(
    x_obstacle, x_ego, v_ego, v_lead, t_follow,
    base_jerk_cost=10.0,
    low_jerk_cost=0.5,
    deadzone_ratio=0.5,
    logistic_k_dist=3.0,
    logistic_k_speed=0.8,
    min_speed_for_closing=0.3,
    distance_smoothing=5.0,
    time_taper=True,
    speed_factor=1.0,
    danger_response_range=(3.0, 6.0),
    danger_jerk_boost=2.0,
    decel_anticipation_factor=0.35
):
    n_steps = len(x_obstacle)
    desired_dist = desired_follow_distance(x_ego, v_lead, t_follow)
    current_gap = x_obstacle - x_ego
    deadzone_threshold = deadzone_ratio * desired_dist
    dist_error = desired_dist - current_gap
    epsilon = 0.2 * deadzone_threshold
    effective_delta = np.where(np.abs(dist_error) < (deadzone_threshold + epsilon),
                               0.0,
                               np.abs(dist_error) - deadzone_threshold)
    k_eff = np.where(dist_error < 0, logistic_k_dist * 0.5, logistic_k_dist)
    dist_activation = 2.0 * (1.0 / (1.0 + np.exp(-k_eff * effective_delta)) - 0.5)
    closing_speed = v_ego - v_lead
    a_ego = np.gradient(v_ego, T_IDXS[:n_steps])
    anticipated_closing = closing_speed + a_ego * decel_anticipation_factor * T_IDXS[:n_steps]
    danger_factor = np.interp(anticipated_closing, danger_response_range, [1.0, danger_jerk_boost])
    danger_factor = np.clip(danger_factor, 1.0, danger_jerk_boost)
    speed_logistic = 1.0 / (1.0 + np.exp(-logistic_k_speed * (closing_speed - min_speed_for_closing)))
    combined_factor = dist_activation * speed_logistic
    jerk_cost_array = low_jerk_cost + combined_factor * (base_jerk_cost - low_jerk_cost)
    if time_taper:
        taper_factors = np.interp(T_IDXS[:n_steps], [0.0, 4.0], [1.3, 1.0])
        jerk_cost_array *= taper_factors
    return jerk_cost_array * speed_factor

def dynamic_lead_pullaway_distance_cost(
    x_ego_i, v_ego_i, x_lead_i, v_lead_i, t_follow_i,
    base_cost=3.0,
    pullaway_dist_margin=7.0,
    pullaway_speed_margin=3.0,
    pullaway_max_factor=1.10,
    exponent=1.0
):
    """
    Modified: If the lead is decelerating (speed_diff <= 0), increase the cost multiplier
    to encourage a more aggressive deceleration.
    """
    desired_dist = desired_follow_distance(v_ego_i, v_lead_i, t_follow_i)
    actual_gap = x_lead_i - x_ego_i
    gap_excess = actual_gap - desired_dist
    speed_diff = v_lead_i - v_ego_i
    # If the lead is decelerating or the gap is negative, increase the cost significantly.
    if gap_excess <= 0 or speed_diff <= 0:
        return base_cost * 1.5
    z_dist = (gap_excess / pullaway_dist_margin) ** exponent
    z_speed = (speed_diff / pullaway_speed_margin) ** exponent
    z = 0.5 * (z_dist + z_speed)
    factor = 1.0 + (pullaway_max_factor - 1.0) * z
    return base_cost * np.clip(factor, 1.0, pullaway_max_factor)

def soft_approach_distance_factor(
    x_obstacle_i, x_ego_i, v_ego_i, v_lead_i, t_follow_i,
    approach_margin=6.0,
    deadzone_margin=1.5,
    max_approach_mult=1.6,
    logistic_k=0.4,
    time_horizon=4.0,
    closing_speed_weight=0.3
):
    def sigmoid(x, k=1.0, x0=0.0):
        return 1.0 / (1.0 + np.exp(-k * (x - x0)))
    desired_dist = desired_follow_distance(v_ego_i, v_lead_i, t_follow_i)
    actual_gap = x_obstacle_i - x_ego_i
    closing_speed = v_ego_i - v_lead_i
    anticipated_gap = actual_gap - closing_speed * time_horizon * closing_speed_weight
    anticipated_gap = np.clip(anticipated_gap, 0.0, None)
    approach_ratio = (anticipated_gap - desired_dist) / (approach_margin + deadzone_margin)
    approach_factor = 1.0 + (max_approach_mult - 1.0) * sigmoid(-approach_ratio,
                                                                   k=logistic_k,
                                                                   x0=deadzone_margin/(approach_margin+deadzone_margin))
    return approach_factor

def dynamic_lead_constraint_weight(
    x_obstacle_i, x_ego_i, v_ego_i, v_lead_i, t_follow_i,
    min_penalty=5.0,
    max_penalty=1e5,
    dist_margin=2.0,
    speed_margin=2.0
):
    desired_dist = desired_follow_distance(v_ego_i, v_lead_i, t_follow_i)
    actual_gap = x_obstacle_i - x_ego_i
    norm_gap_error = ((desired_dist - actual_gap) / (v_ego_i + 10.0))
    penalty_raw = min_penalty + 100.0 * (norm_gap_error ** 2)
    return float(np.clip(penalty_raw, min_penalty, max_penalty))

# -----------------------------------------------------------------------------
# ACADOS Model + OCP Setup
# -----------------------------------------------------------------------------
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
    cost_distance = ((x_obstacle - x_ego) - desired_dist_comfort) / (v_ego + 10.0)

    costs = [
        cost_distance,
        x_ego,
        v_ego,
        a_ego,
        a_ego - prev_a,
        j_ego,
    ]
    ocp.model.cost_y_expr = vertcat(*costs)
    ocp.model.cost_y_expr_e = vertcat(*costs[:-1])

    constraints = vertcat(
        v_ego,
        (a_ego - a_min),
        (a_max - a_ego),
        ((x_obstacle - x_ego) - lead_danger_factor * desired_dist_comfort) / (v_ego + 10.0)
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

# -----------------------------------------------------------------------------
# The Revised MPC Class (Backward Compatible)
# -----------------------------------------------------------------------------
class LongitudinalMpc:
    def __init__(self, mode='acc', dt=DT_MDL):
        self.mode = mode
        self.dt = dt
        self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
        self.reset()
        self.source = SOURCES[2]
        self._approach_factor_last = 1.0

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
        self.set_weights()

    def set_cost_weights(self, cost_weights, constraint_cost_weights):
        W = np.asfortranarray(np.diag(cost_weights))
        for i in range(N):
            W[4,4] = cost_weights[4] * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.3])
            self.solver.cost_set(i, 'W', W)
        self.solver.cost_set(N, 'W', np.copy(W[:COST_E_DIM, :COST_E_DIM]))
        Zl = np.array(constraint_cost_weights)
        for i in range(N):
            self.solver.cost_set(i, 'Zl', Zl)

    def set_weights(self, acceleration_jerk=1.0, danger_jerk=DANGER_ZONE_COST, speed_jerk=1.0,
                    prev_accel_constraint=True, personality=log.LongitudinalPersonality.standard):
        if self.mode == 'acc':
            a_change_cost = acceleration_jerk if prev_accel_constraint else 0.0
            j_cost = J_EGO_COST * acceleration_jerk
            self.base_cost_weights = [
                X_EGO_OBSTACLE_COST,
                X_EGO_COST,
                V_EGO_COST,
                A_EGO_COST,
                a_change_cost,
                j_cost
            ]
            self.base_constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, danger_jerk]
            self.set_cost_weights(self.base_cost_weights, self.base_constraint_cost_weights)
        elif self.mode == 'blended':
            a_change_cost = 40.0 if prev_accel_constraint else 0.0
            cost_weights = [0.0, 0.1, 0.2, 5.0, a_change_cost, 1.0 * acceleration_jerk]
            constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, 50.0]
            self.base_cost_weights = cost_weights
            self.base_constraint_cost_weights = constraint_cost_weights
            self.set_cost_weights(cost_weights, constraint_cost_weights)
        else:
            raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner cost set')

    def set_cur_state(self, v, a):
        v_prev = self.x0[1]
        self.x0[1] = v
        self.x0[2] = a
        if abs(v_prev - v) > 2.0:
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
        a_lead = clip(a_lead, -10.0, 5.0)
        return self.extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau)

    def set_accel_limits(self, min_a, max_a):
        self.cruise_min_a = min_a
        self.max_a = max_a

    def _apply_dynamic_costs(self, chosen_lead_xv):
        base_j_cost = 12.0
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
            logistic_k_dist=3.0,
            logistic_k_speed=0.8,
            distance_smoothing=4.0,
            time_taper=True,
            speed_factor=1.0,
            danger_response_range=(3.0, 6.0),
            danger_jerk_boost=2.0,
            decel_anticipation_factor=0.35
        )
        dist_cost_base = self.base_cost_weights[0]
        a_change_cost = self.base_cost_weights[4]
        for i in range(N):
            W_step = np.diag(self.base_cost_weights)
            W_step[4,4] = a_change_cost * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.3])
            W_step[5,5] = dyn_j_ego_array[i]
            x_ego_i, v_ego_i, _ = self.x_sol[i]
            if i < chosen_lead_xv.shape[0]:
                x_lead_i, v_lead_i = chosen_lead_xv[i]
            else:
                x_lead_i, v_lead_i = chosen_lead_xv[-1]
            t_follow_i = self.params[i,4]
            pullaway_cost = dynamic_lead_pullaway_distance_cost(
                x_ego_i, v_ego_i, x_lead_i, v_lead_i, t_follow_i,
                base_cost=dist_cost_base,
                pullaway_max_factor=1.10,
                exponent=1.0
            )
            approach_factor = soft_approach_distance_factor(
                x_lead_i, x_ego_i, v_ego_i, v_lead_i, t_follow_i,
                approach_margin=5.0,
                max_approach_mult=1.5,
                logistic_k=1.0
            )
            combined_factor = pullaway_cost * approach_factor
            clamped_factor = np.clip(combined_factor, 1.0, 1.3)
            smoothed_factor = 0.95 * self._approach_factor_last + 0.05 * clamped_factor
            self._approach_factor_last = smoothed_factor
            W_step[0,0] = dist_cost_base * smoothed_factor
            last_constraint_penalty = dynamic_lead_constraint_weight(
                x_obstacle_i=self.params[i,2],
                x_ego_i=x_ego_i,
                v_ego_i=v_ego_i,
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
        W_e = np.copy(np.diag(self.base_cost_weights[:COST_E_DIM]))
        self.solver.cost_set(N, 'W', W_e)

    def update(self, lead_one, lead_two, v_cruise, x, v, a, j, radarless_model, t_follow, trafficModeActive,
               personality=log.LongitudinalPersonality.standard):
        v_ego = self.x0[1]
        self.status = lead_one.status or lead_two.status
        lead_xv_0 = self.process_lead(lead_one)
        lead_xv_1 = self.process_lead(lead_two)
        lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor(lead_xv_0[:,1])
        lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor(lead_xv_1[:,1])
        self.params[:,0] = self.cruise_min_a
        self.params[:,1] = self.max_a
        if self.mode == 'acc':
            self.params[:,5] = LEAD_DANGER_FACTOR
            v_lower = v_ego + (T_IDXS * self.cruise_min_a * 1.05)
            v_upper = v_ego + (T_IDXS * self.max_a * 1.05)
            v_cruise_clipped = np.clip(v_cruise * np.ones(N+1), v_lower, v_upper)
            cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, t_follow)
            x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])
            self.source = SOURCES[np.argmin(x_obstacles[0])]
            x[:] = 0.0
            v[:] = 0.0
            a[:] = 0.0
            j[:] = 0.0
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
            raise NotImplementedError(f'Planner mode {self.mode} not recognized in update')
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
        chosen_lead_xv = lead_xv_0
        self._apply_dynamic_costs(chosen_lead_xv)
        self.run()
        lead_probability = lead_one.prob if radarless_model else lead_one.modelProb
        if (np.any(lead_xv_0[FCW_IDXS,0] - self.x_sol[FCW_IDXS,0] < CRASH_DISTANCE)
                and lead_probability > 0.9):
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
                cloudlog.warning(f"Long MPC reset, solution_status: {self.solution_status}")
            self.reset()

if __name__ == "__main__":
    from openpilot.third_party.acados.acados_template import AcadosOcp, AcadosOcpSolver
    ocp = gen_long_ocp()
    AcadosOcpSolver.generate(ocp, json_file=JSON_FILE)
    # Optionally, build with cython:
    # AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)