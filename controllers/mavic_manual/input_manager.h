#ifndef MAVIC_MANUAL_INPUT_MANAGER_H
#define MAVIC_MANUAL_INPUT_MANAGER_H

#include "config.h"
#include "flight_types.h"

typedef struct {
  double filtered[AXIS_CAP];
  int filter_ready[AXIS_CAP];
  double last_raw[AXIS_CAP];
  int have_raw[AXIS_CAP];
  double min_seen[AXIS_CAP];
  double max_seen[AXIS_CAP];
  double abs_delta_avg[AXIS_CAP];
  int direction[AXIS_CAP];
  int moved[AXIS_CAP];
  double invalid_age;
  int prev_button[BUTTON_CAP];
  int prev_arm;
  int prev_takeoff;
  int prev_disarm;
  int prev_pov_arm;
  int prev_pov_takeoff;
  int prev_pov_disarm;
  double disconnect_age;
  int ever_connected;
} InputManager;

void input_init(InputManager *input, int timestep);
void input_update(InputManager *input, const FlightConfig *cfg, PilotInput *out, double dt);
void input_copy_diag(const InputManager *input, AxisDiag *dest, int count);

#endif
