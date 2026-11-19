#ifndef MAVIC_MANUAL_POSITION_CONTROLLER_H
#define MAVIC_MANUAL_POSITION_CONTROLLER_H

#include "config.h"
#include "flight_types.h"

typedef struct {
  LateralMode previous;
  double ix, iy;
  double i_pitch, i_roll;
  double x_sp, y_sp;
  int latched;
  double roll_sp, pitch_sp;
} PositionController;

void position_init(PositionController *ctrl);
void position_reset(PositionController *ctrl);
void limit_tilt_for_speed(double v_forward, double v_left, double v_max, double band, double kv, double *pitch_sp,
                          double *roll_sp);
int position_self_test(void);
void position_update(PositionController *ctrl, const FlightConfig *cfg, const VehicleState *state, LateralMode mode,
                     double stick_roll, double stick_pitch, int latch_now, double dt, ControlTrace *trace);

#endif
