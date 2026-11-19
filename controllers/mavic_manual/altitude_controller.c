#include "altitude_controller.h"

#include "common.h"

#include <math.h>

static double soften_descent(const FlightConfig *cfg, const VehicleState *state, double vz_sp) {
  double height;
  double scale;
  double floor;

  if (!state->ground_valid || vz_sp >= 0.0)
    return vz_sp;
  height = state->z - state->ground_z;
  if (height >= cfg->landing_soft_height)
    return vz_sp;
  scale = clamp_d(height / cfg->landing_soft_height, 0.20, 1.0);
  floor = -fabs(cfg->max_descent_speed) * scale;
  if (vz_sp < floor)
    return floor;
  return vz_sp;
}

void altitude_init(AltitudeController *ctrl, const FlightConfig *cfg) {
  if (!ctrl)
    return;
  pid_init(&ctrl->vz);
  if (cfg) {
    pid_configure(&ctrl->vz, cfg->vz_kp, cfg->vz_ki, 0.0, -fabs(cfg->vz_output_limit), fabs(cfg->vz_output_limit),
                  cfg->vz_i_limit, 0.0);
  }
  ctrl->previous = VERT_OFF;
  ctrl->z_sp = 0.0;
  ctrl->vz_sp = 0.0;
  ctrl->collective = 0.0;
  ctrl->have_latch = 0;
}

void altitude_reset(AltitudeController *ctrl) {
  if (!ctrl)
    return;
  pid_reset(&ctrl->vz);
  ctrl->vz_sp = 0.0;
  ctrl->have_latch = 0;
}

void altitude_update(AltitudeController *ctrl, const FlightConfig *cfg, const VehicleState *state, VertMode mode,
                     double throttle_stick, int latch_now, double takeoff_target_z, double dt, ControlTrace *trace) {
  double vz_target = 0.0;
  double hover;
  double delta;
  double desired;
  double step;
  int vertical_ok;

  if (!trace)
    return;
  if (!ctrl || !cfg || !state) {
    trace->collective = 0.0;
    return;
  }
  if (is_bad(dt) || dt <= 0.0)
    dt = 0.008;

  hover = finite_or(cfg->hover_thrust, 68.5);
  vertical_ok = (state->alt_ok && state->vert_vel_ok) ? 1 : 0;
  throttle_stick = clamp_d(finite_or(throttle_stick, 0.0), -1.0, 1.0);

  if (mode != ctrl->previous) {
    if (mode == VERT_HOLD || mode == VERT_TAKEOFF || latch_now) {
      ctrl->z_sp = state->alt_ok ? state->z : ctrl->z_sp;
      ctrl->have_latch = state->alt_ok ? 1 : 0;
    }
    if (mode == VERT_OFF || mode == VERT_IDLE)
      pid_reset(&ctrl->vz);
    ctrl->previous = mode;
  }
  if (latch_now && state->alt_ok) {
    ctrl->z_sp = state->z;
    ctrl->have_latch = 1;
  }

  if (mode == VERT_OFF) {
    desired = 0.0;
    ctrl->vz_sp = 0.0;
  } else if (mode == VERT_IDLE) {
    desired = cfg->idle_thrust;
    ctrl->vz_sp = 0.0;
  } else if (!vertical_ok || mode == VERT_DEGRADED) {
    desired = (mode == VERT_FAILSAFE || mode == VERT_DEGRADED) ? cfg->degraded_thrust : hover;
    ctrl->vz_sp = slew_toward(ctrl->vz_sp, 0.0, cfg->vz_slew * dt);
  } else if (mode == VERT_TAKEOFF) {
    double climb = fabs(cfg->takeoff_climb_speed);
    ctrl->z_sp = slew_toward(ctrl->z_sp, takeoff_target_z, climb * dt);
    vz_target = clamp_d(cfg->alt_kp * (ctrl->z_sp - state->z), 0.0, climb);
    ctrl->vz_sp = slew_toward(ctrl->vz_sp, vz_target, cfg->vz_slew * dt);
    delta = pid_update(&ctrl->vz, ctrl->vz_sp - state->vz, dt);
    desired = hover + delta;
  } else if (mode == VERT_HOLD) {
    double capped;
    if (!ctrl->have_latch && state->alt_ok) {
      ctrl->z_sp = state->z;
      ctrl->have_latch = 1;
    }
    capped = clamp_d(cfg->alt_kp * (ctrl->z_sp - state->z), -fabs(cfg->max_descent_speed), fabs(cfg->max_ascent_speed));
    ctrl->vz_sp = slew_toward(ctrl->vz_sp, capped, cfg->vz_slew * dt);
    delta = pid_update(&ctrl->vz, ctrl->vz_sp - state->vz, dt);
    desired = hover + delta;
  } else if (mode == VERT_SPEED || mode == VERT_LAND) {
    ctrl->have_latch = 0;
    if (throttle_stick >= 0.0)
      vz_target = throttle_stick * fabs(cfg->max_ascent_speed);
    else
      vz_target = throttle_stick * fabs(cfg->max_descent_speed);
    vz_target = soften_descent(cfg, state, vz_target);
    ctrl->vz_sp = slew_toward(ctrl->vz_sp, vz_target, cfg->vz_slew * dt);
    delta = pid_update(&ctrl->vz, ctrl->vz_sp - state->vz, dt);
    desired = hover + delta;
  } else if (mode == VERT_FAILSAFE) {
    vz_target = -fabs(cfg->failsafe_descent_speed);
    vz_target = soften_descent(cfg, state, vz_target);
    ctrl->vz_sp = slew_toward(ctrl->vz_sp, vz_target, cfg->vz_slew * dt);
    if (state->alt_ok && state->vert_vel_ok) {
      delta = pid_update(&ctrl->vz, ctrl->vz_sp - state->vz, dt);
      desired = hover + delta;
    } else {
      desired = cfg->degraded_thrust;
    }
  } else {
    desired = 0.0;
  }

  if (is_bad(desired))
    desired = 0.0;
  step = fabs(cfg->collective_slew) * dt;
  ctrl->collective = slew_toward(ctrl->collective, desired, step);
  ctrl->collective = clamp_d(ctrl->collective, 0.0, fabs(cfg->motor_command_limit));
  trace->collective = ctrl->collective;
  trace->z_sp = ctrl->z_sp;
  trace->vz_sp = ctrl->vz_sp;
}
