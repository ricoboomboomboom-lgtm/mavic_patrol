#ifndef MAVIC_MANUAL_STATE_ESTIMATOR_H
#define MAVIC_MANUAL_STATE_ESTIMATOR_H

#include "config.h"
#include "flight_types.h"

typedef struct {
  double vx_f, vy_f, vz_f;
  int vel_ready;
  double prev_x, prev_y, prev_z;
  int have_gps;
  double prev_alt;
  int have_alt;
  double ground_z;
  int ground_samples;
  double coast_roll, coast_pitch, coast_yaw;
  double coast_time;
  int attitude_lock;
  double compass_mismatch_time;
} StateEstimator;

void estimator_init(StateEstimator *estimator);
void estimator_update(StateEstimator *estimator, const FlightConfig *cfg, const SensorFrame *frame, VehicleState *state,
                      int airborne, double dt);

#endif
