#ifndef MAVIC_MANUAL_TELEMETRY_H
#define MAVIC_MANUAL_TELEMETRY_H

#include "config.h"
#include "flight_types.h"
#include "input_manager.h"

typedef struct {
  double next_print;
  FlightState last_state;
  char last_block[160];
} Telemetry;

void telemetry_init(Telemetry *telemetry);
void telemetry_update(Telemetry *telemetry, const FlightConfig *cfg, const InputManager *input, const PilotInput *pilot,
                      const SensorFrame *sensors, const VehicleState *state, const SafetyStatus *safety,
                      const ControlTrace *trace, double time);

#endif
