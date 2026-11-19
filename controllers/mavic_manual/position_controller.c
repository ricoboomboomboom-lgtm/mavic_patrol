#include "position_controller.h"

#include "common.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void position_init(PositionController *ctrl) {
  if (!ctrl)
    return;
  memset(ctrl, 0, sizeof(*ctrl));
}

void position_reset(PositionController *ctrl) {
  double roll;
  double pitch;
  if (!ctrl)
    return;
  roll = ctrl->roll_sp;
  pitch = ctrl->pitch_sp;
  memset(ctrl, 0, sizeof(*ctrl));
  ctrl->roll_sp = roll;
  ctrl->pitch_sp = pitch;
}

void limit_tilt_for_speed(double v_forward, double v_left, double v_max, double band, double kv, double *pitch_sp,
                          double *roll_sp) {
  double speed;
  double excess;
  double blend;
  double scale;
  double allow_forward;
  double allow_left;
  double pitch_brake;
  double roll_brake;

  if (!pitch_sp || !roll_sp)
    return;
  if (is_bad(v_forward) || is_bad(v_left))
    return;
  v_max = fabs(v_max);
  band = fabs(band);
  if (band < 0.05)
    band = 0.05;
  speed = hypot(v_forward, v_left);
  excess = speed - v_max;
  if (excess <= 0.0)
    return;
  blend = clamp_d(excess / band, 0.0, 1.0);
  scale = v_max / speed;
  allow_forward = v_forward * scale;
  allow_left = v_left * scale;
  /* +pitch setpoint is nose down, which increases forward speed. */
  pitch_brake = kv * (allow_forward - v_forward);
  roll_brake = -kv * (allow_left - v_left);
  *pitch_sp = ((1.0 - blend) * (*pitch_sp)) + (blend * pitch_brake);
  *roll_sp = ((1.0 - blend) * (*roll_sp)) + (blend * roll_brake);
}

int position_self_test(void) {
  double pitch = 0.40;
  double roll = 0.0;
  limit_tilt_for_speed(6.0, 0.0, 5.0, 0.5, 0.10, &pitch, &roll);
  if (!(pitch < 0.0))
    return 1;
  pitch = 0.20;
  limit_tilt_for_speed(1.0, 0.0, 5.0, 0.5, 0.10, &pitch, &roll);
  if (fabs(pitch - 0.20) > 1e-9)
    return 1;
  return 0;
}

static void decay(double *value, double dt) {
  if (!value)
    return;
  *value *= clamp_d(1.0 - (3.0 * dt), 0.0, 1.0);
}

void position_update(PositionController *ctrl, const FlightConfig *cfg, const VehicleState *state, LateralMode mode,
                     double stick_roll, double stick_pitch, int latch_now, double dt, ControlTrace *trace) {
  double max_roll;
  double max_pitch;
  double pitch_cmd;
  double roll_cmd;
  double slew;
  double vx_sp;
  double vy_sp;
  double v_forward_sp;
  double v_left_sp;
  double speed_cap;

  if (!trace)
    return;
  if (!ctrl || !cfg || !state)
    return;
  if (is_bad(dt) || dt <= 0.0)
    dt = 0.008;

  max_roll = fabs(cfg->max_roll_deg) * M_PI / 180.0;
  max_pitch = fabs(cfg->max_pitch_deg) * M_PI / 180.0;
  slew = fabs(cfg->tilt_slew_deg_s) * M_PI / 180.0 * dt;
  speed_cap = fabs(cfg->max_horizontal_speed);
  stick_roll = clamp_d(finite_or(stick_roll, 0.0), -1.0, 1.0);
  stick_pitch = clamp_d(finite_or(stick_pitch, 0.0), -1.0, 1.0);

  if (mode != ctrl->previous) {
    if (mode != LAT_HOLD) {
      ctrl->ix = 0.0;
      ctrl->iy = 0.0;
      ctrl->latched = 0;
    }
    ctrl->previous = mode;
  }

  if (mode == LAT_MANUAL) {
    /* Stick up / forward is +pitch stick and commands nose-down, which is +pitch. */
    pitch_cmd = stick_pitch * max_pitch;
    roll_cmd = stick_roll * max_roll;
    if (state->pos_ok)
      limit_tilt_for_speed(state->v_forward, state->v_left, speed_cap, cfg->speed_blend_band, cfg->vel_to_tilt, &pitch_cmd,
                           &roll_cmd);
    decay(&ctrl->i_pitch, dt);
    decay(&ctrl->i_roll, dt);
  } else if (mode == LAT_HOLD && state->pos_ok && state->att_ok) {
    double ex;
    double ey;
    double vx_unsat;
    double vy_unsat;
    int saturated;

    if (latch_now || !ctrl->latched) {
      ctrl->x_sp = state->x;
      ctrl->y_sp = state->y;
      ctrl->ix = 0.0;
      ctrl->iy = 0.0;
      ctrl->latched = 1;
    }
    ex = ctrl->x_sp - state->x;
    ey = ctrl->y_sp - state->y;
    if (is_bad(ex) || is_bad(ey)) {
      ex = 0.0;
      ey = 0.0;
    }
    vx_unsat = (cfg->pos_kp * ex) + ctrl->ix;
    vy_unsat = (cfg->pos_kp * ey) + ctrl->iy;
    vx_sp = clamp_d(vx_unsat, -speed_cap, speed_cap);
    vy_sp = clamp_d(vy_unsat, -speed_cap, speed_cap);
    saturated = (fabs(vx_unsat) > speed_cap + 1e-6) || (fabs(vy_unsat) > speed_cap + 1e-6);
    if (!saturated) {
      ctrl->ix = clamp_d(ctrl->ix + (cfg->pos_ki * ex * dt), -fabs(cfg->pos_i_limit), fabs(cfg->pos_i_limit));
      ctrl->iy = clamp_d(ctrl->iy + (cfg->pos_ki * ey * dt), -fabs(cfg->pos_i_limit), fabs(cfg->pos_i_limit));
    } else {
      /* Wind the integrator down while the velocity command is saturated. */
      decay(&ctrl->ix, dt);
      decay(&ctrl->iy, dt);
    }
    v_forward_sp = (cos(state->yaw) * vx_sp) + (sin(state->yaw) * vy_sp);
    v_left_sp = (-sin(state->yaw) * vx_sp) + (cos(state->yaw) * vy_sp);
    if (fabs(v_forward_sp) < 0.2 && fabs(state->v_forward) < 0.4)
      ctrl->i_pitch = clamp_d(ctrl->i_pitch + (cfg->vel_tilt_i * (v_forward_sp - state->v_forward) * dt),
                              -fabs(cfg->vel_tilt_i_limit), fabs(cfg->vel_tilt_i_limit));
    else
      decay(&ctrl->i_pitch, dt);
    if (fabs(v_left_sp) < 0.2 && fabs(state->v_left) < 0.4)
      ctrl->i_roll = clamp_d(ctrl->i_roll + (cfg->vel_tilt_i * (v_left_sp - state->v_left) * dt), -fabs(cfg->vel_tilt_i_limit),
                             fabs(cfg->vel_tilt_i_limit));
    else
      decay(&ctrl->i_roll, dt);
    pitch_cmd = (cfg->vel_to_tilt * (v_forward_sp - state->v_forward)) + ctrl->i_pitch;
    roll_cmd = (-cfg->vel_to_tilt * (v_left_sp - state->v_left)) - ctrl->i_roll;
  } else if (mode == LAT_BRAKE && state->pos_ok && state->att_ok) {
    double yaw_rate = fabs(state->yaw_rate);
    double calm = 1.0;
    decay(&ctrl->i_pitch, dt);
    decay(&ctrl->i_roll, dt);
    /*
     * Body-frame velocity rotates with yaw. Braking hard while still spinning
     * swings the tilt command and, together with a yaw stop, rolls the drone over.
     * Wait until the yaw rate falls, and keep the current tilt meanwhile.
     */
    if (yaw_rate > 0.25)
      calm = clamp_d((0.80 - yaw_rate) / 0.55, 0.0, 1.0);
    if (calm < 0.05) {
      pitch_cmd = ctrl->pitch_sp;
      roll_cmd = ctrl->roll_sp;
    } else {
      pitch_cmd = calm * cfg->vel_to_tilt * (0.0 - state->v_forward);
      roll_cmd = calm * (-cfg->vel_to_tilt * (0.0 - state->v_left));
    }
    ctrl->latched = 0;
  } else {
    pitch_cmd = 0.0;
    roll_cmd = 0.0;
    decay(&ctrl->i_pitch, dt);
    decay(&ctrl->i_roll, dt);
    ctrl->latched = 0;
  }

  pitch_cmd = clamp_d(pitch_cmd, -max_pitch, max_pitch);
  roll_cmd = clamp_d(roll_cmd, -max_roll, max_roll);
  ctrl->pitch_sp = slew_toward(ctrl->pitch_sp, pitch_cmd, slew);
  ctrl->roll_sp = slew_toward(ctrl->roll_sp, roll_cmd, slew);
  trace->pitch_sp = ctrl->pitch_sp;
  trace->roll_sp = ctrl->roll_sp;
}
