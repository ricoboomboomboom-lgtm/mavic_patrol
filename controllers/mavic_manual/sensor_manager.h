#ifndef MAVIC_MANUAL_SENSOR_MANAGER_H
#define MAVIC_MANUAL_SENSOR_MANAGER_H

#include "config.h"
#include "flight_types.h"

void sensors_init(int timestep, const FlightConfig *cfg);
void sensors_read(SensorFrame *frame, const FlightConfig *cfg);
void motors_write(const double signed_velocity[4]);
void leds_set(int left_on, int right_on);
int sensors_motors_ready(void);

#endif
