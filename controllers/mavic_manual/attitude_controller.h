#ifndef MAVIC_MANUAL_ATTITUDE_CONTROLLER_H
#define MAVIC_MANUAL_ATTITUDE_CONTROLLER_H

#include "config.h"
#include "flight_types.h"

typedef struct {
  double roll_trim_i;
  double pitch_trim_i;
  double yaw_trim_i;
  double yaw_sp;
  double yaw_rate_cmd;
  int yaw_latched;
  double yaw_still_time;
  int initialized;
} AttitudeController;

void attitude_init(AttitudeController *ctrl);
void attitude_reset(AttitudeController *ctrl);
void attitude_update(AttitudeController *ctrl, const FlightConfig *cfg, const VehicleState *state, double roll_sp,
                     double pitch_sp, double yaw_stick, int pilot_yaw, int attitude_authority, double dt,
                     ControlTrace *trace);

#endif
