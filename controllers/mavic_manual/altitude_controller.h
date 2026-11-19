#ifndef MAVIC_MANUAL_ALTITUDE_CONTROLLER_H
#define MAVIC_MANUAL_ALTITUDE_CONTROLLER_H

#include "config.h"
#include "flight_types.h"
#include "pid.h"

typedef struct {
  Pid vz;
  VertMode previous;
  double z_sp;
  double vz_sp;
  double collective;
  int have_latch;
} AltitudeController;

void altitude_init(AltitudeController *ctrl, const FlightConfig *cfg);
void altitude_reset(AltitudeController *ctrl);
void altitude_update(AltitudeController *ctrl, const FlightConfig *cfg, const VehicleState *state, VertMode mode,
                     double throttle_stick, int latch_now, double takeoff_target_z, double dt, ControlTrace *trace);

#endif
