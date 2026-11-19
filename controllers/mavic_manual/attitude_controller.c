#include "attitude_controller.h"

#include "common.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void attitude_init(AttitudeController *ctrl) {
  if (!ctrl)
    return;
  ctrl->roll_trim_i = 0.0;
  ctrl->pitch_trim_i = 0.0;
  ctrl->yaw_trim_i = 0.0;
  ctrl->yaw_sp = 0.0;
  ctrl->yaw_rate_cmd = 0.0;
  ctrl->yaw_latched = 0;
  ctrl->yaw_still_time = 0.0;
  ctrl->initialized = 0;
}

void attitude_reset(AttitudeController *ctrl) {
  attitude_init(ctrl);
}

static double trim_integrate(double integral, double error, double ki_scale_dt, double limit, int allow) {
  if (!allow)
    integral *= 0.98;
  else
    integral += error * ki_scale_dt;
  return clamp_d(integral, -limit, limit);
}

void attitude_update(AttitudeController *ctrl, const FlightConfig *cfg, const VehicleState *state, double roll_sp,
                     double pitch_sp, double yaw_stick, int pilot_yaw, int attitude_authority, double dt,
                     ControlTrace *trace) {
  double roll_rate_sp;
  double pitch_rate_sp;
  double yaw_rate_sp;
  double roll_u;
  double pitch_u;
  double yaw_u;
  double yaw_rate_limit;
  double authority;
  int near_level;

  if (!trace)
    return;
  trace->roll_u = 0.0;
  trace->pitch_u = 0.0;
  trace->yaw_u = 0.0;
  trace->yaw_holding = 0;
  if (!ctrl || !cfg || !state)
    return;
  if (is_bad(dt) || dt <= 0.0)
    dt = 0.008;

  if (!ctrl->initialized && state->att_ok) {
    ctrl->yaw_sp = state->yaw;
    ctrl->initialized = 1;
  }

  authority = clamp_d((double)attitude_authority, 0.0, 1.0);
  roll_sp = finite_or(roll_sp, 0.0);
  pitch_sp = finite_or(pitch_sp, 0.0);
  yaw_stick = clamp_d(finite_or(yaw_stick, 0.0), -1.0, 1.0);

  yaw_rate_limit = fabs(cfg->max_yaw_rate_deg) * M_PI / 180.0;
  if (pilot_yaw && yaw_stick != 0.0) {
    double target_rate = -yaw_stick * yaw_rate_limit;
    double rate_step = fabs(cfg->yaw_rate_slew_deg) * M_PI / 180.0 * dt;
    ctrl->yaw_rate_cmd = slew_toward(ctrl->yaw_rate_cmd, target_rate, rate_step);
    ctrl->yaw_latched = 0;
    ctrl->yaw_still_time = 0.0;
    if (state->att_ok)
      ctrl->yaw_sp = state->yaw;
  } else {
    /* Stopping a yaw is slower than starting one. A hard yaw brake while the
       vehicle is still tilted couples into roll and can flip it. */
    double stop_slew = fabs(cfg->yaw_rate_slew_deg) * 0.45;
    double rate_step = stop_slew * M_PI / 180.0 * dt;
    ctrl->yaw_rate_cmd = slew_toward(ctrl->yaw_rate_cmd, 0.0, rate_step);
    if (state->att_ok && state->rates_ok && fabs(state->yaw_rate) < cfg->yaw_hold_rate &&
        fabs(ctrl->yaw_rate_cmd) < cfg->yaw_hold_rate) {
      ctrl->yaw_still_time += dt;
      if (!ctrl->yaw_latched && ctrl->yaw_still_time >= cfg->yaw_lock_time) {
        ctrl->yaw_sp = state->yaw;
        ctrl->yaw_latched = 1;
      }
    } else if (!ctrl->yaw_latched) {
      ctrl->yaw_still_time = 0.0;
    }
  }

  if (!state->att_ok) {
    ctrl->roll_trim_i = 0.0;
    ctrl->pitch_trim_i = 0.0;
    ctrl->yaw_trim_i = 0.0;
    trace->roll_sp = roll_sp;
    trace->pitch_sp = pitch_sp;
    trace->yaw_sp = ctrl->yaw_sp;
    trace->yaw_rate_sp = 0.0;
    return;
  }

  /* Outer loop: angle error becomes a rate command. Inner loop tracks that rate. */
  roll_rate_sp = cfg->roll_angle_kp * (roll_sp - state->roll);
  pitch_rate_sp = cfg->pitch_angle_kp * (pitch_sp - state->pitch);
  roll_rate_sp = clamp_d(roll_rate_sp, -12.0, 12.0);
  pitch_rate_sp = clamp_d(pitch_rate_sp, -12.0, 12.0);

  if (ctrl->yaw_latched && fabs(ctrl->yaw_rate_cmd) < cfg->yaw_hold_rate) {
    yaw_rate_sp = cfg->yaw_angle_kp * wrap_pi(ctrl->yaw_sp - state->yaw);
    trace->yaw_holding = 1;
  } else {
    yaw_rate_sp = ctrl->yaw_rate_cmd;
  }
  yaw_rate_sp = clamp_d(yaw_rate_sp, -yaw_rate_limit, yaw_rate_limit);

  /*
   * Roll and pitch: positive output corrects a positive angle (right wing down
   * or nose down). The rate error is measured_rate - commanded_rate.
   * Yaw: positive output commands nose-left, so the rate error is reversed.
   */
  roll_u = cfg->roll_rate_kp * ((state->rates_ok ? state->roll_rate : 0.0) - roll_rate_sp);
  pitch_u = cfg->pitch_rate_kp * ((state->rates_ok ? state->pitch_rate : 0.0) - pitch_rate_sp);
  yaw_u = cfg->yaw_rate_kp * (yaw_rate_sp - (state->rates_ok ? state->yaw_rate : 0.0));

  near_level = (fabs(roll_sp) < 0.08 && fabs(pitch_sp) < 0.08 && fabs(state->roll) < 0.35 && fabs(state->pitch) < 0.35);
  ctrl->roll_trim_i = trim_integrate(ctrl->roll_trim_i, state->roll - roll_sp, dt, cfg->roll_trim_i_limit, near_level);
  ctrl->pitch_trim_i = trim_integrate(ctrl->pitch_trim_i, state->pitch - pitch_sp, dt, cfg->pitch_trim_i_limit, near_level);
  if (trace->yaw_holding)
    ctrl->yaw_trim_i = trim_integrate(ctrl->yaw_trim_i, wrap_pi(ctrl->yaw_sp - state->yaw), dt, cfg->yaw_trim_i_limit, 1);
  else
    ctrl->yaw_trim_i *= 0.98;

  roll_u += cfg->roll_trim_ki * ctrl->roll_trim_i;
  pitch_u += cfg->pitch_trim_ki * ctrl->pitch_trim_i;
  yaw_u += cfg->yaw_trim_ki * ctrl->yaw_trim_i;

  roll_u = clamp_d(roll_u, -fabs(cfg->roll_output_limit), fabs(cfg->roll_output_limit));
  pitch_u = clamp_d(pitch_u, -fabs(cfg->pitch_output_limit), fabs(cfg->pitch_output_limit));
  yaw_u = clamp_d(yaw_u, -fabs(cfg->yaw_output_limit), fabs(cfg->yaw_output_limit));

  trace->roll_u = roll_u * authority;
  trace->pitch_u = pitch_u * authority;
  trace->yaw_u = yaw_u * authority;
  trace->roll_sp = roll_sp;
  trace->pitch_sp = pitch_sp;
  trace->yaw_sp = ctrl->yaw_sp;
  trace->yaw_rate_sp = yaw_rate_sp;
}
