#include "sensor_manager.h"

#include "common.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <webots/accelerometer.h>
#include <webots/altimeter.h>
#include <webots/compass.h>
#include <webots/device.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/led.h>
#include <webots/motor.h>
#include <webots/nodes.h>
#include <webots/robot.h>
#include <webots/supervisor.h>

/*
 * Device names are the DEF/name fields of projects/robots/dji/mavic/protos/Mavic2Pro.proto
 * (Webots R2025a). Accelerometer and altimeter are not in that proto; the manual
 * world adds them on bodySlot, where Webots names them "accelerometer" and "altimeter".
 */

static WbDeviceTag imu_tag;
static WbDeviceTag gyro_tag;
static WbDeviceTag gps_tag;
static WbDeviceTag compass_tag;
static WbDeviceTag accel_tag;
static WbDeviceTag altimeter_tag;
static WbDeviceTag motor_fl;
static WbDeviceTag motor_fr;
static WbDeviceTag motor_rl;
static WbDeviceTag motor_rr;
static WbDeviceTag led_left;
static WbDeviceTag led_right;
static WbNodeRef self_node;
static int contact_enabled;
static int motors_ready;

static WbDeviceTag find_device(const char *name, WbNodeType type) {
  int count;
  int i;

  count = wb_robot_get_number_of_devices();
  for (i = 0; i < count; ++i) {
    WbDeviceTag tag = wb_robot_get_device_by_index(i);
    const char *device_name = wb_device_get_name(tag);
    if (!device_name)
      continue;
    if (strcmp(device_name, name) == 0 && wb_device_get_node_type(tag) == type)
      return tag;
  }
  return 0;
}

static void enable_motor(WbDeviceTag tag, const char *name) {
  if (!tag) {
    printf("mavic_manual: missing motor '%s'\n", name);
    return;
  }
  wb_motor_set_position(tag, INFINITY);
  wb_motor_set_velocity(tag, 0.0);
}

void sensors_init(int timestep, const FlightConfig *cfg) {
  int period = timestep > 0 ? timestep : 8;

  (void)cfg;
  imu_tag = find_device("inertial unit", WB_NODE_INERTIAL_UNIT);
  gyro_tag = find_device("gyro", WB_NODE_GYRO);
  gps_tag = find_device("gps", WB_NODE_GPS);
  compass_tag = find_device("compass", WB_NODE_COMPASS);
  accel_tag = find_device("accelerometer", WB_NODE_ACCELEROMETER);
  altimeter_tag = find_device("altimeter", WB_NODE_ALTIMETER);
  motor_fl = find_device("front left propeller", WB_NODE_ROTATIONAL_MOTOR);
  motor_fr = find_device("front right propeller", WB_NODE_ROTATIONAL_MOTOR);
  motor_rl = find_device("rear left propeller", WB_NODE_ROTATIONAL_MOTOR);
  motor_rr = find_device("rear right propeller", WB_NODE_ROTATIONAL_MOTOR);
  led_left = find_device("front left led", WB_NODE_LED);
  led_right = find_device("front right led", WB_NODE_LED);

  if (imu_tag)
    wb_inertial_unit_enable(imu_tag, period);
  if (gyro_tag)
    wb_gyro_enable(gyro_tag, period);
  if (gps_tag)
    wb_gps_enable(gps_tag, period);
  if (compass_tag)
    wb_compass_enable(compass_tag, period);
  if (accel_tag)
    wb_accelerometer_enable(accel_tag, period);
  if (altimeter_tag)
    wb_altimeter_enable(altimeter_tag, period);

  enable_motor(motor_fl, "front left propeller");
  enable_motor(motor_fr, "front right propeller");
  enable_motor(motor_rl, "rear left propeller");
  enable_motor(motor_rr, "rear right propeller");
  motors_ready = (motor_fl && motor_fr && motor_rl && motor_rr) ? 1 : 0;

  self_node = wb_supervisor_node_get_self();
  contact_enabled = 0;
  if (self_node) {
    wb_supervisor_node_enable_contact_points_tracking(self_node, period, 1);
    contact_enabled = 1;
  }

  printf("mavic_manual: devices imu=%d gyro=%d gps=%d compass=%d accel=%d altimeter=%d motors=%d leds=%d supervisor=%d\n",
         imu_tag ? 1 : 0, gyro_tag ? 1 : 0, gps_tag ? 1 : 0, compass_tag ? 1 : 0, accel_tag ? 1 : 0,
         altimeter_tag ? 1 : 0, motors_ready, (led_left && led_right) ? 1 : 0, contact_enabled);
}

int sensors_motors_ready(void) {
  return motors_ready;
}

static int finite3(const double *v) {
  return v && !is_bad(v[0]) && !is_bad(v[1]) && !is_bad(v[2]);
}

void sensors_read(SensorFrame *frame, const FlightConfig *cfg) {
  const double *values;

  if (!frame)
    return;
  memset(frame, 0, sizeof(*frame));
  frame->imu_present = imu_tag ? 1 : 0;
  frame->gyro_present = gyro_tag ? 1 : 0;
  frame->gps_present = gps_tag ? 1 : 0;
  frame->compass_present = compass_tag ? 1 : 0;
  frame->accel_present = accel_tag ? 1 : 0;
  frame->altimeter_present = altimeter_tag ? 1 : 0;
  frame->motors_present = motors_ready;
  frame->led_present = (led_left && led_right) ? 1 : 0;
  frame->supervisor = contact_enabled;

  if (imu_tag) {
    values = wb_inertial_unit_get_roll_pitch_yaw(imu_tag);
    if (finite3(values) && fabs(values[0]) < 4.0 && fabs(values[1]) < 4.0 && fabs(values[2]) < 7.0) {
      frame->roll = values[0];
      frame->pitch = values[1];
      frame->yaw = values[2];
      frame->imu_finite = 1;
    }
  }

  if (gyro_tag) {
    values = wb_gyro_get_values(gyro_tag);
    if (finite3(values) && fabs(values[0]) < 40.0 && fabs(values[1]) < 40.0 && fabs(values[2]) < 40.0) {
      frame->roll_rate = values[0];
      frame->pitch_rate = values[1];
      frame->yaw_rate = values[2];
      frame->gyro_finite = 1;
    }
  }

  if (gps_tag) {
    values = wb_gps_get_values(gps_tag);
    if (finite3(values) && fabs(values[0]) < 5000.0 && fabs(values[1]) < 5000.0 && fabs(values[2]) < 5000.0) {
      frame->gps_x = values[0];
      frame->gps_y = values[1];
      frame->gps_z = values[2];
      frame->gps_finite = 1;
    }
    values = wb_gps_get_speed_vector(gps_tag);
    if (finite3(values) && fabs(values[0]) < 80.0 && fabs(values[1]) < 80.0 && fabs(values[2]) < 80.0) {
      frame->gps_vx = values[0];
      frame->gps_vy = values[1];
      frame->gps_vz = values[2];
      frame->gps_speed_finite = 1;
    }
  }

  if (compass_tag) {
    values = wb_compass_get_values(compass_tag);
    if (finite3(values)) {
      frame->compass_x = values[0];
      frame->compass_y = values[1];
      frame->compass_z = values[2];
      frame->compass_finite = 1;
    }
  }

  if (accel_tag) {
    values = wb_accelerometer_get_values(accel_tag);
    if (finite3(values) && fabs(values[0]) < 80.0 && fabs(values[1]) < 80.0 && fabs(values[2]) < 80.0) {
      frame->accel_x = values[0];
      frame->accel_y = values[1];
      frame->accel_z = values[2];
      frame->accel_finite = 1;
    }
  }

  if (altimeter_tag) {
    double altitude = wb_altimeter_get_value(altimeter_tag);
    if (!is_bad(altitude) && fabs(altitude) < 5000.0) {
      frame->altimeter = altitude;
      frame->altimeter_finite = 1;
    }
  }

  frame->ground_contact = 0;
  frame->contact_count = 0;
  if (contact_enabled && self_node) {
    int count = 0;
    int i;
    double floor_z = cfg ? cfg->ground_contact_z : 0.08;
    WbContactPoint *points = wb_supervisor_node_get_contact_points(self_node, 1, &count);
    /*
     * The returned array is owned by the Webots library. Freeing it crashes
     * the controller on the next step.
     */
    if (count < 0)
      count = 0;
    frame->contact_count = count;
    for (i = 0; i < count && points; ++i) {
      if (!is_bad(points[i].point[2]) && points[i].point[2] <= floor_z)
        frame->ground_contact = 1;
    }
  }
}

void motors_write(const double signed_velocity[4]) {
  if (!motors_ready || !signed_velocity)
    return;
  wb_motor_set_velocity(motor_fl, finite_or(signed_velocity[0], 0.0));
  wb_motor_set_velocity(motor_fr, finite_or(signed_velocity[1], 0.0));
  wb_motor_set_velocity(motor_rl, finite_or(signed_velocity[2], 0.0));
  wb_motor_set_velocity(motor_rr, finite_or(signed_velocity[3], 0.0));
}

void leds_set(int left_on, int right_on) {
  if (led_left)
    wb_led_set(led_left, left_on ? 1 : 0);
  if (led_right)
    wb_led_set(led_right, right_on ? 1 : 0);
}
