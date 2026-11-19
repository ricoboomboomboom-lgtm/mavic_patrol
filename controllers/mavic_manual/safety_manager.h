#ifndef MAVIC_MANUAL_SAFETY_MANAGER_H
#define MAVIC_MANUAL_SAFETY_MANAGER_H

#include "config.h"
#include "flight_types.h"

typedef struct {
  FlightState state;
  double state_time;
  double gate_time;
  double hold_ready_time;
  double contact_time;
  double heuristic_time;
  double handover_time;
  double shutdown_time;
  int spun;
  int left_ground;
  int block_arm_until_release;
  int latch_position_pulse;
  int latch_altitude_pulse;
  int fail_reason;
  LateralMode lateral;
  double takeoff_target_z;
  char detail[160];
} SafetyManager;

void safety_init(SafetyManager *safety);
void safety_update(SafetyManager *safety, const FlightConfig *cfg, const PilotInput *input, const SensorFrame *sensors,
                   const VehicleState *state, double dt, SafetyStatus *status);

#endif
