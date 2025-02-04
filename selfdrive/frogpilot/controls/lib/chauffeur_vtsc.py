#!/usr/bin/env python3
"""
Vision Turn Speed Controller Script

This script computes a planned speed trajectory for turning maneuvers based on model
predictions. It includes additional logic to handle curvy roads—delaying braking,
increasing deceleration when needed, and boosting acceleration on short straight
segments—without changing the lateral acceleration targets. This patched version is
100% backward compatible.
"""

import math
import numpy as np
import cereal.messaging as messaging

from openpilot.common.conversions import Conversions as CV
from openpilot.common.numpy_fast import clip
from openpilot.selfdrive.modeld.constants import ModelConstants

CRUISING_SPEED = 5.0  # m/s


def nonlinear_lat_accel(v_ego_ms: float, turn_aggressiveness: float = 1.0) -> float:
    """
    Compute lateral acceleration limit based on speed and an aggressiveness factor.
    """
    v_ego_mph = v_ego_ms * CV.MS_TO_MPH
    base = 1.5
    span = 2.2
    center = 35.0
    k = 0.10

    lat_acc = base + span / (1.0 + math.exp(-k * (v_ego_mph - center)))
    lat_acc = min(lat_acc, 2.8)
    return lat_acc * turn_aggressiveness


def margin_time_fn(v_ego_ms: float) -> float:
    """
    Returns a 'margin time' used in backward-pass speed planning.
    The faster you go, the more margin time is used.
    """
    v_low = 0.0
    t_low = 1.0
    v_med = 15.0     # ~34 mph
    t_med = 3.0
    v_high = 31.3    # ~70 mph
    t_high = 5.0

    if v_ego_ms <= v_low:
        return t_low
    elif v_ego_ms >= v_high:
        return t_high
    elif v_ego_ms <= v_med:
        ratio = (v_ego_ms - v_low) / (v_med - v_low)
        return t_low + ratio * (t_med - t_low)
    else:
        ratio = (v_ego_ms - v_med) / (v_high - v_med)
        return t_med + ratio * (t_high - t_med)


def find_apexes(curv_array: np.ndarray, threshold: float = 5e-5) -> list:
    """
    Identify indices where curvature spikes above a threshold and is a local maximum.
    """
    apex_indices = []
    for i in range(1, len(curv_array) - 1):
        if (
            curv_array[i] > threshold
            and curv_array[i] >= curv_array[i + 1]
            and curv_array[i] > curv_array[i - 1]
        ):
            apex_indices.append(i)
    return apex_indices


def dynamic_decel_scale(v_ego_ms: float) -> float:
    """
    Originally was 4.0 -> 1.0 from ~5 m/s to ~25 m/s.
    Now we double it for lower speeds: 8.0 -> 1.0.
    """
    min_speed = 5.0   # below this speed => max scaling
    max_speed = 25.0  # above this speed => 1.0
    if v_ego_ms <= min_speed:
        return 8.0
    elif v_ego_ms >= max_speed:
        return 2.0
    else:
        # Linear interpolation from 8.0 -> 1.0
        ratio = (v_ego_ms - min_speed) / (max_speed - min_speed)
        return 8.0 + (1.0 - 8.0) * ratio


def dynamic_jerk_scale(v_ego_ms: float) -> float:
    """
    Same doubling logic for jerk scaling.
    """
    return dynamic_decel_scale(v_ego_ms)


class VisionTurnSpeedController:
    def __init__(
        self,
        turn_smoothing_alpha=0.3,
        reaccel_alpha=0.2,
        low_lat_acc=0.20,
        high_lat_acc=0.40,
        max_decel=3.0,
        max_jerk=6.0
    ):
        self.turn_smoothing_alpha = turn_smoothing_alpha
        self.reaccel_alpha = reaccel_alpha
        self.LOW_LAT_ACC = low_lat_acc
        self.HIGH_LAT_ACC = high_lat_acc

        self.MAX_DECEL = max_decel
        self.MAX_JERK = max_jerk

        self.current_accel = 0.0
        self.prev_target_speed = 0.0

        self.planned_speeds = np.zeros(33, dtype=float)
        self.sm = messaging.SubMaster(['modelV2'])

        self.prev_v_cruise_cluster = 0.0

        # NEW: Attribute to record if the recent road segment was curvy.
        self.last_is_curvy = False

    def reset(self, v_ego: float) -> None:
        """
        Reset internal state so the controller is ready for fresh speed planning.
        """
        self.prev_target_speed = v_ego
        self.current_accel = 0.0
        self.planned_speeds[:] = v_ego

    def update(self, v_ego: float, v_cruise_cluster: float, turn_aggressiveness=1.0) -> float:
        """
        Main entry point for the turn speed controller logic. Called every cycle.
        """
        self.sm.update()
        modelData = self.sm['modelV2']

        # If below a certain speed, just reset and do nothing fancy.
        if v_ego < CRUISING_SPEED:
            self.reset(v_ego)
            self.prev_v_cruise_cluster = v_cruise_cluster
            return v_ego

        orientation_rate_raw = modelData.orientationRate.z
        velocity_pred_raw = modelData.velocity.x

        MIN_POINTS = 3
        if (
            orientation_rate_raw is None or velocity_pred_raw is None
            or len(orientation_rate_raw) < MIN_POINTS
            or len(velocity_pred_raw) < MIN_POINTS
        ):
            # Fallback if the model data isn't available.
            raw_target = self._single_step_fallback(v_ego, 0.0, turn_aggressiveness)
        else:
            orientation_rate = np.abs(np.array(orientation_rate_raw, dtype=float))
            velocity_pred = np.array(velocity_pred_raw, dtype=float)

            n_points = min(len(orientation_rate), len(velocity_pred))
            # Interpolate or slice to exactly 33 points.
            if n_points < 33:
                src_indices = np.linspace(0, n_points - 1, n_points)
                dst_indices = np.linspace(0, n_points - 1, 33)
                orientation_rate_33 = np.interp(dst_indices, src_indices, orientation_rate[:n_points])
                velocity_pred_33 = np.interp(dst_indices, src_indices, velocity_pred[:n_points])
            else:
                orientation_rate_33 = orientation_rate[:33]
                velocity_pred_33 = velocity_pred[:33]

            times_33 = np.array(ModelConstants.T_IDXS[:33], dtype=float)

            # Check if the driver just bumped up.
            is_bump_up = (v_cruise_cluster > self.prev_v_cruise_cluster)

            self.planned_speeds = self._plan_speed_trajectory(
                orientation_rate_33,
                velocity_pred_33,
                times_33,
                init_speed=self.prev_target_speed,
                turn_aggressiveness=turn_aggressiveness,
                skip_accel_limit=is_bump_up
            )
            raw_target = self.planned_speeds[0]

        # Always clamp raw_target to at most v_cruise_cluster.
        raw_target = min(raw_target, v_cruise_cluster)

        dt = 0.05  # ~20Hz update rate
        final_target_speed = None

        # If the driver just increased the set speed, skip symmetrical jerk-limiting on acceleration.
        if v_cruise_cluster > self.prev_v_cruise_cluster:
            final_target_speed = min(raw_target, v_cruise_cluster)
            self.current_accel = 0.0  # Optional reset.
        
        if final_target_speed is None:
            # Compute an extra boost factor for acceleration ramp-up.
            planned_max = np.max(self.planned_speeds)
            if v_cruise_cluster > self.prev_target_speed:
                ratio = (planned_max - self.prev_target_speed) / max((v_cruise_cluster - self.prev_target_speed), 1e-3)
                ratio = clip(ratio, 0.0, 1.0)
                boost_factor = 1.0 + ratio
            else:
                boost_factor = 1.0

            # NEW: On a curvy road, apply an additional boost to help regain speed on short straights.
            if self.last_is_curvy:
                boost_factor *= 1.2

            # Use dynamic scaling as before, but now allow extra boost on the positive side.
            scale_decel = dynamic_decel_scale(v_ego)
            scale_jerk = dynamic_jerk_scale(v_ego)

            # Compute the acceleration command (m/s^2).
            accel_cmd = (raw_target - self.prev_target_speed) / dt

            # Allow a higher positive acceleration if exiting a curve;
            # deceleration (negative accel) remains capped by MAX_DECEL * scale_decel.
            pos_limit = self.MAX_DECEL * scale_decel * boost_factor
            neg_limit = self.MAX_DECEL * scale_decel
            accel_cmd = clip(accel_cmd, -neg_limit, pos_limit)

            # Jerk-limit the change in acceleration.
            max_delta = (self.MAX_JERK * scale_jerk * boost_factor) * dt
            accel_diff = accel_cmd - self.current_accel
            if accel_diff > max_delta:
                self.current_accel += max_delta
            elif accel_diff < -max_delta:
                self.current_accel -= max_delta
            else:
                self.current_accel = accel_cmd

            final_target_speed = self.prev_target_speed + self.current_accel * dt

        # Always clamp the final target speed to the dash set speed.
        final_target_speed = min(final_target_speed, v_cruise_cluster)

        self.prev_target_speed = final_target_speed
        self.prev_v_cruise_cluster = v_cruise_cluster
        return final_target_speed

    def _plan_speed_trajectory(
        self,
        orientation_rate: np.ndarray,
        velocity_pred: np.ndarray,
        times: np.ndarray,
        init_speed: float,
        turn_aggressiveness: float,
        skip_accel_limit: bool = False
    ) -> np.ndarray:
        """
        Build a future speed plan based on curvature. If skip_accel_limit=True,
        we skip *positive* acceleration capping in the forward pass but still
        allow negative acceleration (slowing down) to prepare for turns.
        """
        n = len(orientation_rate)
        eps = 1e-9
        dt_array = np.diff(times)

        # Compute curvature for each lookahead point.
        curvature = np.array([
            orientation_rate[i] / max(velocity_pred[i], eps)
            for i in range(n)
        ], dtype=float)

        # NEW: Compute a "curviness" metric using the mean absolute curvature.
        curviness = np.mean(np.abs(curvature))
        curvy_threshold = 0.02  # (1/m) Tunable threshold.
        is_curvy = curviness > curvy_threshold
        self.last_is_curvy = is_curvy  # Save flag for use in update().

        # Set multipliers based on whether the road is curvy.
        if is_curvy:
            # On a curvy road: delay braking by reducing deceleration/spool times,
            # and shorten the deceleration margin window.
            decel_factor_multiplier = 0.7
            spool_factor_multiplier = 0.7
            margin_multiplier = 0.7
        else:
            decel_factor_multiplier = 1.0
            spool_factor_multiplier = 1.0
            margin_multiplier = 1.0

        # Compute safe speeds for each future point based on lateral acceleration limits.
        safe_speeds = np.zeros(n, dtype=float)
        for i in range(n):
            lat_acc_limit = nonlinear_lat_accel(velocity_pred[i], turn_aggressiveness)
            c = max(curvature[i], 1e-9)
            safe_speeds[i] = math.sqrt(lat_acc_limit / c) if c > 1e-9 else 70.0
            safe_speeds[i] = clip(safe_speeds[i], 0.0, 70.0)
            # PATCH: Reduce safe speed by 7% if within ~30–45 mph (13.4–20.1 m/s)
            if 13.4 <= safe_speeds[i] <= 20.1:
                safe_speeds[i] *= 0.93

        # Identify apex indices and shape speeds around them.
        apex_idxs = find_apexes(curvature, threshold=5e-5)
        planned = safe_speeds.copy()
        for apex_i in apex_idxs:
            apex_speed = planned[apex_i]

            # Base factors for shaping speeds around the apex.
            base_decel_factor = 0.15
            base_spool_factor = 0.08
            # Adjust factors based on curvy conditions.
            decel_factor = base_decel_factor * decel_factor_multiplier
            spool_factor = base_spool_factor * spool_factor_multiplier

            decel_sec = velocity_pred[apex_i] * decel_factor
            spool_sec = velocity_pred[apex_i] * spool_factor

            decel_start = self._find_time_index(times, times[apex_i] - decel_sec)
            spool_start = self._find_time_index(times, times[apex_i] - spool_sec)
            spool_end = self._find_time_index(times, times[apex_i] + spool_sec, clip_high=True)

            if spool_start > decel_start:
                v_decel_start = planned[decel_start]
                steps_decel = spool_start - decel_start
                for idx in range(decel_start, spool_start):
                    f = (idx - decel_start) / float(steps_decel)
                    planned[idx] = v_decel_start * (1 - f) + apex_speed * f

            for idx in range(spool_start, apex_i + 1):
                planned[idx] = min(planned[idx], apex_speed)

            if spool_end > apex_i:
                steps_spool = spool_end - apex_i
                v_spool_end = planned[spool_end - 1]
                for idx in range(apex_i, spool_end):
                    f = (idx - apex_i) / float(steps_spool)
                    spool_val = apex_speed * (1 - f) + v_spool_end * f
                    planned[idx] = min(planned[idx], spool_val)

        # Margin-based deceleration shaping: backward pass.
        new_planned = planned.copy()
        margin_t = margin_time_fn(init_speed) * margin_multiplier

        for i in range(n - 2, -1, -1):
            j = self._find_time_index(times, times[i] + margin_t, clip_high=True)
            if j <= i:
                continue
            dt_ij = times[j] - times[i]
            if dt_ij < 0.001:
                continue
            v_future = new_planned[j]
            err = new_planned[i] - v_future
            desired_acc = clip(err / dt_ij, -self.MAX_DECEL, self.MAX_DECEL)
            feasible_speed = v_future - desired_acc * dt_ij
            new_planned[i] = min(new_planned[i], feasible_speed, planned[i])

        # Standard backward pass for deceleration limiting.
        dt_len = len(dt_array)
        for i in range(n - 2, -1, -1):
            dt_i = dt_array[i] if i < dt_len else 0.05
            v_next = new_planned[i + 1]
            err = new_planned[i] - v_next
            desired_acc = clip(err / dt_i, -self.MAX_DECEL, self.MAX_DECEL)
            feasible_speed = v_next - desired_acc * dt_i
            new_planned[i] = min(new_planned[i], feasible_speed, planned[i])

        # Forward pass: limit acceleration changes between points.
        new_planned[0] = min(new_planned[0], planned[0])
        dt_0 = dt_array[0] if dt_len > 0 else 0.05
        err0 = new_planned[0] - init_speed

        if skip_accel_limit and err0 > 0:
            accel0 = clip(err0 / dt_0, -self.MAX_DECEL, 9999.0)
        else:
            accel0 = clip(err0 / dt_0, -self.MAX_DECEL, self.MAX_DECEL)
        new_planned[0] = init_speed + accel0 * dt_0

        for i in range(1, n):
            dt_i = dt_array[i - 1] if (i - 1 < dt_len) else 0.05
            v_prev = new_planned[i - 1]
            err = new_planned[i] - v_prev

            if skip_accel_limit and err > 0:
                desired_acc = clip(err / dt_i, -self.MAX_DECEL, 9999.0)
            else:
                desired_acc = clip(err / dt_i, -self.MAX_DECEL, self.MAX_DECEL)
            feasible_speed = v_prev + desired_acc * dt_i
            new_planned[i] = min(new_planned[i], feasible_speed, planned[i])

        return new_planned

    def _find_time_index(self, times: np.ndarray, target_time: float, clip_high=False) -> int:
        """
        Helper to find an index in 'times' that is closest to 'target_time'.
        If clip_high=True, clamp to the last index if target_time exceeds times[-1].
        """
        n = len(times)
        if target_time <= times[0]:
            return 0
        if target_time >= times[-1] and clip_high:
            return n - 1
        for i in range(n - 1):
            if times[i] <= target_time < times[i + 1]:
                # Return whichever index is closer.
                if (target_time - times[i]) < (times[i + 1] - target_time):
                    return i
                else:
                    return i + 1
        return n - 1 if clip_high else n - 2

    def _single_step_fallback(self, v_ego, curvature, turn_aggressiveness):
        """
        Fallback strategy when model data is not available.
        Clamps speed based on lateral acceleration limits.
        """
        lat_acc = nonlinear_lat_accel(v_ego, turn_aggressiveness)
        c = max(curvature, 1e-9)
        safe_speed = math.sqrt(lat_acc / c) if c > 1e-9 else 70.0
        safe_speed = clip(safe_speed, 0.0, 70.0)

        current_lat_acc = curvature * (v_ego ** 2)
        if current_lat_acc > self.LOW_LAT_ACC:
            if current_lat_acc < self.HIGH_LAT_ACC:
                alpha = 0.1
            else:
                alpha = self.turn_smoothing_alpha
            raw_target = alpha * self.prev_target_speed + (1 - alpha) * safe_speed
        else:
            alpha = self.reaccel_alpha
            raw_target = alpha * self.prev_target_speed + (1 - alpha) * safe_speed

        return raw_target