/*
 * Plain data shared across the controller. Device tags stay inside the
 * modules that talk to Webots.
 */

#ifndef MAVIC_MANUAL_FLIGHT_TYPES_H
#define MAVIC_MANUAL_FLIGHT_TYPES_H

#define AXIS_CAP 16
#define BUTTON_CAP 64
#define POV_CAP 4

typedef enum {
  FS_INITIALIZATION = 0,
  FS_DIAGNOSTIC,
  FS_READY,
  FS_DISARMED,
  FS_ARMING,
  FS_ARMED,
  FS_TAKEOFF,
  FS_FLIGHT,
  FS_POSITION_HOLD,
  FS_LANDING,
  FS_FAILSAFE,
  FS_LANDED,
  FS_COUNT
} FlightState;

typedef enum {
  LAT_LEVEL = 0,
  LAT_MANUAL,
  LAT_BRAKE,
  LAT_HOLD
} LateralMode;

typedef enum {
  VERT_OFF = 0,
  VERT_IDLE,
  VERT_TAKEOFF,
  VERT_HOLD,
  VERT_SPEED,
  VERT_LAND,
  VERT_FAILSAFE,
  VERT_DEGRADED
} VertMode;

/* Distinct failsafe causes. A USB loss is not treated as a GPS loss. */
#define FAIL_NONE 0
#define FAIL_USB 1
#define FAIL_ATTITUDE 2
#define FAIL_ALTITUDE 4

#define FAULT_IMU 1
#define FAULT_GYRO 2
#define FAULT_GPS 4
#define FAULT_ALTIMETER 8
#define FAULT_COMPASS 16
#define FAULT_ACCEL 32
#define FAULT_MOTOR 64

typedef struct {
  int connected;
  int valid;
  double age_s;
  int axis_count;
  int pov_count;
  double raw[AXIS_CAP];
  double unit[AXIS_CAP];
  int button[BUTTON_CAP];
  int button_overflow;
  int pov[POV_CAP];
  int pressed_count;
  char model[128];
  double roll;
  double pitch;
  double yaw;
  double throttle;
  int map_complete;
  int arm_rise;
  int takeoff_rise;
  int disarm_rise;
  int arm_held;
  int takeoff_held;
  int disarm_held;
  int safe_sticks;
} PilotInput;

typedef struct {
  double raw;
  double min_seen;
  double max_seen;
  double mean_abs_delta;
  int direction;
  int moved;
} AxisDiag;

typedef struct {
  int imu_present;
  int gyro_present;
  int gps_present;
  int compass_present;
  int accel_present;
  int altimeter_present;
  int motors_present;
  int led_present;
  int supervisor;
  int contact_count;
  int ground_contact;
  double gps_x, gps_y, gps_z;
  double gps_vx, gps_vy, gps_vz;
  int gps_finite;
  int gps_speed_finite;
  double roll, pitch, yaw;
  int imu_finite;
  double roll_rate, pitch_rate, yaw_rate;
  int gyro_finite;
  double compass_x, compass_y, compass_z;
  int compass_finite;
  double accel_x, accel_y, accel_z;
  int accel_finite;
  double altimeter;
  int altimeter_finite;
} SensorFrame;

typedef struct {
  double x, y, z;
  double vx, vy, vz;
  double roll, pitch, yaw;
  double roll_rate, pitch_rate, yaw_rate;
  double v_forward, v_left;
  double ground_z;
  int ground_valid;
  int att_ok;
  int rates_ok;
  int pos_ok;
  int alt_ok;
  int vert_vel_ok;
  int using_altimeter;
  int compass_disagrees;
  double compass_yaw;
  int fault_bits;
  double attitude_coast_s;
} VehicleState;

typedef struct {
  FlightState state;
  LateralMode lateral;
  VertMode vertical;
  int fail_reason;
  int motors_live;
  int left_ground;
  int latch_position;
  int latch_altitude;
  int pilot_sticks_enabled;
  double attitude_authority;
  double takeoff_target_z;
  const char *arm_block;
  char detail[160];
} SafetyStatus;

typedef struct {
  double roll_sp;
  double pitch_sp;
  double yaw_sp;
  double yaw_rate_sp;
  double z_sp;
  double vz_sp;
  double collective;
  double roll_u;
  double pitch_u;
  double yaw_u;
  double motor[4];
  int yaw_holding;
} ControlTrace;

const char *flight_state_name(FlightState state);
const char *lateral_mode_name(LateralMode mode);
const char *vertical_mode_name(VertMode mode);

#endif
