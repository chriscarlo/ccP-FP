#!/usr/bin/env python3
import math
import numpy as np
from openpilot.common.numpy_fast import clip, interp
import cereal.messaging as messaging
from openpilot.common.conversions import Conversions as CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.simple_kalman import KF1D
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC, LEAD_ACCEL_TAU
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, V_CRUISE_UNSET, CONTROL_N, get_speed_error
from openpilot.common.swaglog import cloudlog

LON_MPC_STEP = 0.2
A_CRUISE_MIN = -6.0
A_CRUISE_MAX_VALS = [4.2, 3.5, 2.8, 1.6, 0.8]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40., 50.]
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]
ALLOW_THROTTLE_THRESHOLD = 0.5
MIN_ALLOW_THROTTLE_SPEED = 2.5
LEAD_KALMAN_SPEED, LEAD_KALMAN_ACCEL = 0, 1


def get_max_accel(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)


def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3


def get_accel_from_plan(speeds, accels, action_t=DT_MDL, vEgoStopping=0.05):
  if len(speeds) == CONTROL_N:
    v_now = speeds[0]
    a_now = accels[0]
    v_target = interp(action_t, CONTROL_N_T_IDX, speeds)
    a_target = 2 * (v_target - v_now) / action_t - a_now
    v_target_1sec = interp(action_t + 1.0, CONTROL_N_T_IDX, speeds)
  else:
    v_target = 0.0
    v_target_1sec = 0.0
    a_target = 0.0
  should_stop = (v_target < vEgoStopping and v_target_1sec < vEgoStopping)
  return a_target, should_stop


def lead_kf(v_lead: float, dt: float = 0.05):
  assert 0.01 < dt < 0.2
  A = [[1.0, dt], [0.0, 1.0]]
  C = [1.0, 0.0]
  dts = [0.01 * i for i in range(1, 21)]
  K0 = [
    0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689, 0.21372394,
    0.22761098, 0.24069424, 0.253096, 0.26491023, 0.27621103, 0.28705801,
    0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
    0.35353899, 0.36200124
  ]
  K1 = [
    0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
    0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
    0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
    0.26393339, 0.26278425
  ]
  K = [[interp(dt, dts, K0)], [interp(dt, dts, K1)]]
  kf = KF1D([[v_lead], [0.0]], A, C, K)
  return kf


class Lead:
  def __init__(self):
    self.dRel = 0.0
    self.yRel = 0.0
    self.vLead = 0.0
    self.aLead = 0.0
    self.vLeadK = 0.0
    self.aLeadK = 0.0
    self.aLeadTau = LEAD_ACCEL_TAU
    self.prob = 0.0
    self.status = False
    self.kf: KF1D | None = None

  def reset(self):
    self.status = False
    self.kf = None
    self.aLeadTau = LEAD_ACCEL_TAU

  def update(self, dRel: float, yRel: float, vLead: float, aLead: float, prob: float):
    self.dRel = dRel
    self.yRel = yRel
    self.vLead = vLead
    self.aLead = aLead
    self.prob = prob
    self.status = True
    if self.kf is None:
      self.kf = lead_kf(self.vLead)
    else:
      self.kf.update(self.vLead)
    self.vLeadK = float(self.kf.x[LEAD_KALMAN_SPEED][0])
    self.aLeadK = float(self.kf.x[LEAD_KALMAN_ACCEL][0])
    if abs(self.aLeadK) < 0.5:
      self.aLeadTau = LEAD_ACCEL_TAU
    else:
      self.aLeadTau *= 0.9


def _scale_t_follow(original_t_follow: float, v_ego: float) -> float:
  max_extra = 0.2
  scale_speed = 30.0
  extra = max_extra * min(v_ego, scale_speed) / scale_speed
  return original_t_follow + extra


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True
    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.v_model_error = 0.0
    self.lead_one = Lead()
    self.lead_two = Lead()
    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory_full = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0

  @staticmethod
  def parse_model(model_msg, model_error, v_ego, taco_tune):
    if (
      len(model_msg.position.x) == ModelConstants.IDX_N
      and len(model_msg.velocity.x) == ModelConstants.IDX_N
      and len(model_msg.acceleration.x) == ModelConstants.IDX_N
    ):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x) - model_error * T_IDXS_MPC
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x) - model_error
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    if taco_tune:
      max_lat_accel = interp(v_ego, [5, 10, 20], [1.5, 2.0, 3.0])
      curvatures = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.orientationRate.z) / np.clip(v, 0.3, 100.0)
      max_v = np.sqrt(max_lat_accel / (np.abs(curvatures) + 1e-3)) - 2.0
      v = np.minimum(max_v, v)
    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0
    return x, v, a, j, throttle_prob

  def update(self, classic_model, radarless_model, sm, frogpilot_toggles):
    self.mpc.mode = "blended" if sm["controlsState"].experimentalMode else "acc"
    if len(sm["carControl"].orientationNED) == 3:
      accel_coast = get_coast_accel(sm["carControl"].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX
    v_ego = sm["carState"].vEgo
    v_cruise_kph = min(sm["controlsState"].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    v_cruise_initialized = sm["controlsState"].vCruise != V_CRUISE_UNSET
    long_control_off = sm["controlsState"].longControlState == LongCtrlState.off
    force_slow_decel = sm["controlsState"].forceDecel
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm["controlsState"].enabled
    reset_state = reset_state or not v_cruise_initialized
    prev_accel_constraint = not (reset_state or sm["carState"].standstill)
    accel_limits = [sm["frogpilotPlan"].minAcceleration, sm["frogpilotPlan"].maxAcceleration]
    if self.mpc.mode == "acc":
      accel_limits = [A_CRUISE_MIN, get_max_accel(v_ego)]
      accel_limits_turns = [A_CRUISE_MIN, get_max_accel(v_ego)]
    else:
      accel_limits = [ACCEL_MIN, ACCEL_MAX]
      accel_limits_turns = [ACCEL_MIN, ACCEL_MAX]
    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = clip(sm["carState"].aEgo, accel_limits[0], accel_limits[1])
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    self.v_model_error = get_speed_error(sm["modelV2"], v_ego)
    x, v, a, j, throttle_prob = self.parse_model(sm["modelV2"], self.v_model_error, v_ego, frogpilot_toggles.taco_tune)
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED
    if not self.allow_throttle and not classic_model:
      clipped_accel_coast = max(accel_coast, accel_limits_turns[0])
      clipped_accel_coast_interp = interp(
        v_ego,
        [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED * 2],
        [accel_limits_turns[1], clipped_accel_coast],
      )
      accel_limits_turns[1] = min(accel_limits_turns[1], clipped_accel_coast_interp)
    if force_slow_decel:
      v_cruise = 0.0
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired + 0.05)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired - 0.05)
    if radarless_model:
      model_leads = list(sm["modelV2"].leadsV3)
      lead_states = [self.lead_one, self.lead_two]
      for index in range(len(lead_states)):
        if len(model_leads) > index:
          model_lead = model_leads[index]
          distance_offset = (
            max(
              frogpilot_toggles.increased_stopped_distance + min(10 - v_ego, 0),
              0,
            )
            if not sm["frogpilotCarState"].trafficModeActive
            else 0
          )
          speed_based_offset = min(v_ego, 30.0) * 0.1667
          distance_offset += speed_based_offset
          lead_states[index].update(
            model_lead.x[0] - distance_offset,
            model_lead.y[0],
            model_lead.v[0],
            model_lead.a[0],
            model_lead.prob,
          )
        else:
          lead_states[index].reset()
    else:
      self.lead_one = sm["radarState"].leadOne
      self.lead_two = sm["radarState"].leadTwo
    accelerationJerk = sm["frogpilotPlan"].accelerationJerk
    dangerJerk = sm["frogpilotPlan"].dangerJerk
    speedJerk = sm["frogpilotPlan"].speedJerk
    jerk_factor = 1.2
    accelerationJerk = sm["frogpilotPlan"].accelerationJerk
    dangerJerk       = sm["frogpilotPlan"].dangerJerk
    speedJerk        = sm["frogpilotPlan"].speedJerk

    # example scale factor
    jerk_factor = 1.2
    accelerationJerk *= jerk_factor
    dangerJerk       *= jerk_factor
    speedJerk        *= jerk_factor

    prev_accel_constraint = not (reset_state or sm["carState"].standstill)

    self.mpc.set_weights(accelerationJerk,
                        dangerJerk,
                        speedJerk,
                        prev_accel_constraint,
                        personality=sm["controlsState"].personality)

    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)

    v_ego = sm["carState"].vEgo
    
    # scale the plan's tFollow if desired
    t_follow_scaled = _scale_t_follow(sm["frogpilotPlan"].tFollow, v_ego)
    
    self.mpc.update(
      self.lead_one,
      self.lead_two,
      sm["frogpilotPlan"].vCruise,
      x,
      v,
      a,
      j,
      radarless_model,
      t_follow_scaled,  # pass the scaled version
      sm["frogpilotCarState"].trafficModeActive,
      personality=sm["controlsState"].personality,
    )

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.a_desired_trajectory_full = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)
    self.fcw = self.mpc.crash_cnt > 2 and not sm["carState"].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")
    a_prev = self.a_desired
    self.a_desired = float(interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x += self.dt * (self.a_desired + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message("longitudinalPlan")
    plan_send.valid = sm.all_checks(service_list=["carState", "controlsState"])
    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime["modelV2"]
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime["modelV2"]
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time
    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()
    longitudinalPlan.hasLead = self.lead_one.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw
    action_t = self.CP.longitudinalActuatorDelay + DT_MDL
    a_target, should_stop = get_accel_from_plan(
      longitudinalPlan.speeds,
      longitudinalPlan.accels,
      action_t=action_t,
      vEgoStopping=self.CP.vEgoStopping,
    )
    if self.mpc.mode == "blended":
      e2e_speeds = list(sm["modelV2"].velocity.x)[:CONTROL_N]
      e2e_accels = list(sm["modelV2"].acceleration.x)[:CONTROL_N]
      a_target_e2e, should_stop_e2e = get_accel_from_plan(
        e2e_speeds,
        e2e_accels,
        action_t=action_t,
        vEgoStopping=self.CP.vEgoStopping,
      )
      a_target = min(a_target, a_target_e2e)
      should_stop = should_stop or should_stop_e2e
    longitudinalPlan.aTarget = a_target
    longitudinalPlan.shouldStop = should_stop
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = self.allow_throttle
    pm.send("longitudinalPlan", plan_send)
