#include "input_manager.h"

#include "common.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <webots/joystick.h>

static int pov_matches(const PilotInput *out, int pov_index, int pov_value) {
  if (pov_index < 0 || pov_value < 0)
    return 0;
  if (pov_index >= out->pov_count || pov_index >= POV_CAP)
    return 0;
  return out->pov[pov_index] == pov_value;
}

static double normalize_raw(double raw, double vmin, double vmax, double neutral) {
  if (!(vmax > neutral + 1.0) || !(neutral > vmin + 1.0)) {
    vmin = -32768.0;
    vmax = 32767.0;
    neutral = 0.0;
  }
  raw = clamp_d(raw, vmin, vmax);
  if (raw >= neutral)
    return (raw - neutral) / (vmax - neutral);
  return (raw - neutral) / (neutral - vmin);
}

static double shape_channel(InputManager *input, int axis, double raw, double vmin, double vmax, double neutral,
                            double deadzone, double expo, int invert, double tau, double dt) {
  double unit;
  double filtered;
  double sign;
  double mag;

  if (axis < 0 || axis >= AXIS_CAP)
    return 0.0;

  unit = normalize_raw(raw, vmin, vmax, neutral);
  if (invert)
    unit = -unit;
  unit = apply_expo(unit, expo);
  if (!input->filter_ready[axis]) {
    input->filtered[axis] = unit;
    input->filter_ready[axis] = 1;
  } else {
    input->filtered[axis] = lowpass(input->filtered[axis], unit, tau, dt);
  }
  filtered = input->filtered[axis];
  deadzone = clamp_d(deadzone, 0.0, 0.9);
  if (fabs(filtered) <= deadzone)
    return 0.0;
  sign = (filtered < 0.0) ? -1.0 : 1.0;
  mag = (fabs(filtered) - deadzone) / (1.0 - deadzone);
  return sign * clamp_d(mag, 0.0, 1.0);
}

static void clear_outputs(PilotInput *out) {
  out->roll = 0.0;
  out->pitch = 0.0;
  out->yaw = 0.0;
  out->throttle = 0.0;
  out->valid = 0;
  out->safe_sticks = 0;
  out->arm_rise = 0;
  out->takeoff_rise = 0;
  out->disarm_rise = 0;
}

void input_init(InputManager *input, int timestep) {
  int i;

  if (!input)
    return;
  memset(input, 0, sizeof(*input));
  for (i = 0; i < AXIS_CAP; ++i) {
    input->min_seen[i] = 1.0e9;
    input->max_seen[i] = -1.0e9;
  }
  wb_joystick_enable(timestep > 0 ? timestep : 8);
}

void input_copy_diag(const InputManager *input, AxisDiag *dest, int count) {
  int i;
  if (!input || !dest)
    return;
  if (count > AXIS_CAP)
    count = AXIS_CAP;
  for (i = 0; i < count; ++i) {
    dest[i].raw = 0.0;
    dest[i].min_seen = input->min_seen[i];
    dest[i].max_seen = input->max_seen[i];
    dest[i].mean_abs_delta = input->abs_delta_avg[i];
    dest[i].direction = input->direction[i];
    dest[i].moved = input->moved[i];
  }
}

void input_update(InputManager *input, const FlightConfig *cfg, PilotInput *out, double dt) {
  int axis_count;
  int pov_count;
  int i;
  int button;
  int arm_now;
  int takeoff_now;
  int disarm_now;
  const char *model;
  double raw_roll = 0.0;
  double raw_pitch = 0.0;
  double raw_yaw = 0.0;
  double raw_throttle = 0.0;

  if (!input || !cfg || !out)
    return;
  if (is_bad(dt) || dt <= 0.0)
    dt = 0.008;

  memset(out, 0, sizeof(*out));
  for (i = 0; i < POV_CAP; ++i)
    out->pov[i] = -1;

  out->connected = wb_joystick_is_connected() ? 1 : 0;
  model = wb_joystick_get_model();
  if (model)
    snprintf(out->model, sizeof(out->model), "%s", model);

  axis_count = wb_joystick_get_number_of_axes();
  pov_count = wb_joystick_get_number_of_povs();
  if (axis_count < 0)
    axis_count = 0;
  if (axis_count > AXIS_CAP)
    axis_count = AXIS_CAP;
  if (pov_count < 0)
    pov_count = 0;
  if (pov_count > POV_CAP)
    pov_count = POV_CAP;
  out->axis_count = axis_count;
  out->pov_count = pov_count;

  if (!out->connected || axis_count <= 0) {
    input->invalid_age += dt;
    out->age_s = input->invalid_age;
    clear_outputs(out);
    out->connected = wb_joystick_is_connected() ? 1 : 0;
    out->axis_count = axis_count;
    /* Drain the button list so a later reconnect cannot replay an old edge. */
    while (wb_joystick_get_pressed_button() >= 0) {
    }
    return;
  }

  input->ever_connected = 1;

  for (i = 0; i < axis_count; ++i) {
    int sample = wb_joystick_get_axis_value(i);
    double raw = (double)sample;
    double delta;

    if (sample < -32768 || sample > 32767) {
      out->valid = 0;
      clear_outputs(out);
      out->connected = 1;
      out->axis_count = axis_count;
      out->age_s = cfg->usb_timeout_s;
      return;
    }
    out->raw[i] = raw;
    delta = 0.0;
    if (input->have_raw[i])
      delta = raw - input->last_raw[i];
    input->last_raw[i] = raw;
    input->have_raw[i] = 1;
    if (raw < input->min_seen[i])
      input->min_seen[i] = raw;
    if (raw > input->max_seen[i])
      input->max_seen[i] = raw;
    if ((input->max_seen[i] - input->min_seen[i]) > 1000.0)
      input->moved[i] = 1;
    input->abs_delta_avg[i] = lowpass(input->abs_delta_avg[i], fabs(delta), 0.30, dt);
    if (delta > 250.0)
      input->direction[i] = 1;
    else if (delta < -250.0)
      input->direction[i] = -1;
    else
      input->direction[i] = 0;
  }

  for (i = 0; i < pov_count; ++i)
    out->pov[i] = wb_joystick_get_pov_value(i);

  memset(out->button, 0, sizeof(out->button));
  out->pressed_count = 0;
  out->button_overflow = 0;
  button = wb_joystick_get_pressed_button();
  while (button >= 0) {
    if (button < BUTTON_CAP) {
      if (!out->button[button]) {
        out->button[button] = 1;
        ++out->pressed_count;
      }
    } else {
      out->button_overflow = 1;
    }
    button = wb_joystick_get_pressed_button();
  }

  out->map_complete = (cfg->roll_axis >= 0 && cfg->pitch_axis >= 0 && cfg->yaw_axis >= 0 && cfg->throttle_axis >= 0 &&
                       cfg->roll_axis < axis_count && cfg->pitch_axis < axis_count && cfg->yaw_axis < axis_count &&
                       cfg->throttle_axis < axis_count);

  if (out->map_complete) {
    raw_roll = out->raw[cfg->roll_axis];
    raw_pitch = out->raw[cfg->pitch_axis];
    raw_yaw = out->raw[cfg->yaw_axis];
    raw_throttle = out->raw[cfg->throttle_axis];
    out->roll = shape_channel(input, cfg->roll_axis, raw_roll, cfg->roll_min, cfg->roll_max, cfg->roll_neutral,
                              cfg->roll_deadzone, cfg->roll_expo, cfg->roll_invert, cfg->stick_filter_tau, dt);
    out->pitch = shape_channel(input, cfg->pitch_axis, raw_pitch, cfg->pitch_min, cfg->pitch_max, cfg->pitch_neutral,
                               cfg->pitch_deadzone, cfg->pitch_expo, cfg->pitch_invert, cfg->stick_filter_tau, dt);
    out->yaw = shape_channel(input, cfg->yaw_axis, raw_yaw, cfg->yaw_min, cfg->yaw_max, cfg->yaw_neutral, cfg->yaw_deadzone,
                             cfg->yaw_expo, cfg->yaw_invert, cfg->stick_filter_tau, dt);
    out->throttle = shape_channel(input, cfg->throttle_axis, raw_throttle, cfg->throttle_min, cfg->throttle_max,
                                  cfg->throttle_neutral, cfg->throttle_deadzone, cfg->throttle_expo, cfg->throttle_invert,
                                  cfg->throttle_filter_tau, dt);
    out->valid = 1;
    input->invalid_age = 0.0;
  } else {
    clear_outputs(out);
    out->connected = 1;
    out->axis_count = axis_count;
    out->map_complete = 0;
    out->valid = 0;
    input->invalid_age += dt;
  }
  out->age_s = input->invalid_age;

  /*
   * Semantic sticks, after calibration:
   *   roll +1    stick right
   *   pitch +1   stick forward (nose-down command)
   *   yaw +1     stick right (clockwise)
   *   throttle +1 climb
   * The throttle stick does not spring back. Neutral is the calibrated value.
   */
  out->safe_sticks = (out->valid && out->roll == 0.0 && out->pitch == 0.0 && out->yaw == 0.0 && out->throttle == 0.0);

  arm_now = 0;
  takeoff_now = 0;
  disarm_now = 0;
  if (cfg->arm_button >= 0 && cfg->arm_button < BUTTON_CAP && out->button[cfg->arm_button])
    arm_now = 1;
  if (cfg->takeoff_button >= 0 && cfg->takeoff_button < BUTTON_CAP && out->button[cfg->takeoff_button])
    takeoff_now = 1;
  if (cfg->disarm_button >= 0 && cfg->disarm_button < BUTTON_CAP && out->button[cfg->disarm_button])
    disarm_now = 1;
  if (pov_matches(out, cfg->arm_pov, cfg->arm_pov_value))
    arm_now = 1;
  if (pov_matches(out, cfg->takeoff_pov, cfg->takeoff_pov_value))
    takeoff_now = 1;
  if (pov_matches(out, cfg->disarm_pov, cfg->disarm_pov_value))
    disarm_now = 1;

  out->arm_held = arm_now;
  out->takeoff_held = takeoff_now;
  out->disarm_held = disarm_now;
  out->arm_rise = (arm_now && !input->prev_arm) ? 1 : 0;
  out->takeoff_rise = (takeoff_now && !input->prev_takeoff) ? 1 : 0;
  out->disarm_rise = (disarm_now && !input->prev_disarm) ? 1 : 0;
  input->prev_arm = arm_now;
  input->prev_takeoff = takeoff_now;
  input->prev_disarm = disarm_now;
  memcpy(input->prev_button, out->button, sizeof(out->button));

  if (!out->valid) {
    out->roll = 0.0;
    out->pitch = 0.0;
    out->yaw = 0.0;
    out->throttle = 0.0;
    out->safe_sticks = 0;
  }
}
