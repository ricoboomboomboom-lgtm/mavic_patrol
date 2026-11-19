#include "state_estimator.h"

#include "common.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

void estimator_init(StateEstimator *estimator) {
  if (!estimator)
    return;
  memset(estimator, 0, sizeof(*estimator));
}

static void body_velocity(double yaw, double vx, double vy, double *forward, double *left) {
  double c = cos(yaw);
  double s = sin(yaw);
  /* Yaw 0 faces +X. +Y is left. Positive yaw is counterclockwise from above. */
  *forward = (c * vx) + (s * vy);
  *left = (-s * vx) + (c * vy);
}

void estimator_update(StateEstimator *estimator, const FlightConfig *cfg, const SensorFrame *frame, VehicleState *state,
                      int airborne, double dt) {
  const double tau = 0.06;
  int gps_velocity = 0;

  (void)cfg;
  if (!estimator || !frame || !state)
    return;
  if (is_bad(dt) || dt <= 0.0 || dt > 0.05)
    dt = 0.008;

  memset(state, 0, sizeof(*state));

  if (!frame->imu_present || !frame->imu_finite)
    state->fault_bits |= FAULT_IMU;
  if (!frame->gyro_present || !frame->gyro_finite)
    state->fault_bits |= FAULT_GYRO;
  if (!frame->gps_present || !frame->gps_finite)
    state->fault_bits |= FAULT_GPS;
  if (!frame->altimeter_present || !frame->altimeter_finite)
    state->fault_bits |= FAULT_ALTIMETER;
  if (!frame->compass_present || !frame->compass_finite)
    state->fault_bits |= FAULT_COMPASS;
  if (!frame->accel_present || !frame->accel_finite)
    state->fault_bits |= FAULT_ACCEL;
  if (!frame->motors_present)
    state->fault_bits |= FAULT_MOTOR;

  state->rates_ok = frame->gyro_finite ? 1 : 0;
  if (state->rates_ok) {
    state->roll_rate = frame->roll_rate;
    state->pitch_rate = frame->pitch_rate;
    state->yaw_rate = frame->yaw_rate;
  }

  if (frame->imu_finite) {
    state->roll = frame->roll;
    state->pitch = frame->pitch;
    state->yaw = wrap_pi(frame->yaw);
    state->att_ok = 1;
    estimator->attitude_lock = 2;
    estimator->coast_time = 0.0;
    estimator->coast_roll = state->roll;
    estimator->coast_pitch = state->pitch;
    estimator->coast_yaw = state->yaw;
  } else if (state->rates_ok && estimator->attitude_lock >= 1) {
    /* Gyro coast for at most 2 s after the inertial unit drops out. */
    estimator->attitude_lock = 1;
    estimator->coast_time += dt;
    estimator->coast_roll += state->roll_rate * dt;
    estimator->coast_pitch += state->pitch_rate * dt;
    estimator->coast_yaw = wrap_pi(estimator->coast_yaw + (state->yaw_rate * dt));
    if (estimator->coast_time < 2.0) {
      state->roll = estimator->coast_roll;
      state->pitch = estimator->coast_pitch;
      state->yaw = estimator->coast_yaw;
      state->att_ok = 1;
    } else {
      estimator->attitude_lock = 0;
      state->att_ok = 0;
    }
  }
  state->attitude_coast_s = estimator->coast_time;

  if (frame->compass_finite) {
    /* Health check only. Control yaw comes from the inertial unit. */
    state->compass_yaw = wrap_pi(atan2(frame->compass_y, frame->compass_x));
    if (state->att_ok && fabs(wrap_pi(state->compass_yaw - state->yaw)) > 0.6)
      estimator->compass_mismatch_time += dt;
    else
      estimator->compass_mismatch_time = 0.0;
    state->compass_disagrees = (estimator->compass_mismatch_time > 1.0) ? 1 : 0;
  } else {
    state->compass_yaw = state->yaw;
  }

  if (frame->gps_finite) {
    state->x = frame->gps_x;
    state->y = frame->gps_y;
    state->pos_ok = 1;
    if (frame->gps_speed_finite) {
      state->vx = frame->gps_vx;
      state->vy = frame->gps_vy;
      state->vz = frame->gps_vz;
      gps_velocity = 1;
      state->vert_vel_ok = 1;
    } else if (estimator->have_gps) {
      state->vx = (frame->gps_x - estimator->prev_x) / dt;
      state->vy = (frame->gps_y - estimator->prev_y) / dt;
      state->vz = (frame->gps_z - estimator->prev_z) / dt;
      gps_velocity = 1;
      state->vert_vel_ok = 1;
    }
    estimator->prev_x = frame->gps_x;
    estimator->prev_y = frame->gps_y;
    estimator->prev_z = frame->gps_z;
    estimator->have_gps = 1;
  } else {
    estimator->have_gps = 0;
  }

  if (frame->altimeter_finite) {
    state->z = frame->altimeter;
    state->alt_ok = 1;
    state->using_altimeter = 1;
    if (!gps_velocity && estimator->have_alt) {
      state->vz = (frame->altimeter - estimator->prev_alt) / dt;
      state->vert_vel_ok = !is_bad(state->vz);
    }
    estimator->prev_alt = frame->altimeter;
    estimator->have_alt = 1;
  } else if (frame->gps_finite) {
    state->z = frame->gps_z;
    state->alt_ok = 1;
    state->using_altimeter = 0;
    estimator->have_alt = 0;
  } else {
    estimator->have_alt = 0;
  }

  if (state->pos_ok || state->vert_vel_ok) {
    if (!estimator->vel_ready) {
      estimator->vx_f = state->vx;
      estimator->vy_f = state->vy;
      estimator->vz_f = state->vz;
      estimator->vel_ready = 1;
    } else {
      estimator->vx_f = lowpass(estimator->vx_f, state->vx, tau, dt);
      estimator->vy_f = lowpass(estimator->vy_f, state->vy, tau, dt);
      estimator->vz_f = lowpass(estimator->vz_f, state->vz, tau, dt);
    }
    state->vx = estimator->vx_f;
    state->vy = estimator->vy_f;
    state->vz = estimator->vz_f;
  }

  if (is_bad(state->vx))
    state->vx = 0.0;
  if (is_bad(state->vy))
    state->vy = 0.0;
  if (is_bad(state->vz)) {
    state->vz = 0.0;
    state->vert_vel_ok = 0;
  }

  if (state->att_ok && state->pos_ok)
    body_velocity(state->yaw, state->vx, state->vy, &state->v_forward, &state->v_left);

  if (!airborne && state->alt_ok && fabs(state->vz) < 0.05) {
    if (estimator->ground_samples == 0)
      estimator->ground_z = state->z;
    else
      estimator->ground_z = (0.98 * estimator->ground_z) + (0.02 * state->z);
    if (estimator->ground_samples < 10000)
      ++estimator->ground_samples;
  }
  state->ground_z = estimator->ground_z;
  state->ground_valid = (estimator->ground_samples >= 20) ? 1 : 0;
}
