#ifndef MAVIC_MANUAL_CONFIG_H
#define MAVIC_MANUAL_CONFIG_H

typedef struct {
  int roll_axis;
  int pitch_axis;
  int yaw_axis;
  int throttle_axis;
  int roll_invert;
  int pitch_invert;
  int yaw_invert;
  int throttle_invert;
  double roll_min, roll_max, roll_neutral;
  double pitch_min, pitch_max, pitch_neutral;
  double yaw_min, yaw_max, yaw_neutral;
  double throttle_min, throttle_max, throttle_neutral;
  double roll_deadzone, pitch_deadzone, yaw_deadzone, throttle_deadzone;
  double roll_expo, pitch_expo, yaw_expo, throttle_expo;
  double stick_filter_tau;
  double throttle_filter_tau;
  int arm_button;
  int takeoff_button;
  int disarm_button;
  int arm_pov;
  int arm_pov_value;
  int takeoff_pov;
  int takeoff_pov_value;
  int disarm_pov;
  int disarm_pov_value;
  double usb_timeout_s;
  double handover_valid_s;

  double max_roll_deg;
  double max_pitch_deg;
  double max_horizontal_speed;
  double speed_blend_band;
  double max_ascent_speed;
  double max_descent_speed;
  double failsafe_descent_speed;
  double takeoff_climb_speed;
  double takeoff_height;
  double max_yaw_rate_deg;
  double tilt_slew_deg_s;
  double vz_slew;
  double yaw_rate_slew_deg;
  double brake_speed_enter_hold;
  double brake_hold_time;
  double landing_enter_height;
  double landing_soft_height;
  double liftoff_clearance;
  double ground_contact_z;
  double ground_contact_time;
  double contact_vz_max;
  double arm_max_tilt_deg;
  double arming_time;
  double armed_timeout;
  int diagnostic_mode;
  double telemetry_period;
  double diagnostic_period;

  double hover_thrust;
  double idle_thrust;
  double motor_command_limit;
  double collective_slew;
  double degraded_thrust;

  double roll_angle_kp;
  double roll_rate_kp;
  double roll_trim_ki;
  double roll_trim_i_limit;
  double roll_output_limit;
  double pitch_angle_kp;
  double pitch_rate_kp;
  double pitch_trim_ki;
  double pitch_trim_i_limit;
  double pitch_output_limit;
  double yaw_angle_kp;
  double yaw_rate_kp;
  double yaw_trim_ki;
  double yaw_trim_i_limit;
  double yaw_output_limit;
  double yaw_hold_rate;
  double yaw_lock_time;

  double vel_to_tilt;
  double vel_tilt_i;
  double vel_tilt_i_limit;
  double pos_kp;
  double pos_ki;
  double pos_i_limit;

  double alt_kp;
  double vz_kp;
  double vz_ki;
  double vz_i_limit;
  double vz_output_limit;

  double expected_timestep_ms;
} FlightConfig;

void config_set_defaults(FlightConfig *cfg);
int config_load(FlightConfig *cfg, const char *path, char *error, int error_len);
void config_resolve_path(char *dest, int dest_len);

#endif
