#include "config.h"

#include "common.h"

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include <webots/robot.h>

typedef enum { CFG_INT = 0, CFG_DOUBLE = 1 } CfgKind;

typedef struct {
  const char *key;
  CfgKind kind;
  int offset;
} CfgField;

#define FLD(member, kind) {#member, kind, (int)offsetof(FlightConfig, member)}

static const CfgField FIELDS[] = {
    FLD(roll_axis, CFG_INT),
    FLD(pitch_axis, CFG_INT),
    FLD(yaw_axis, CFG_INT),
    FLD(throttle_axis, CFG_INT),
    FLD(roll_invert, CFG_INT),
    FLD(pitch_invert, CFG_INT),
    FLD(yaw_invert, CFG_INT),
    FLD(throttle_invert, CFG_INT),
    FLD(roll_min, CFG_DOUBLE),
    FLD(roll_max, CFG_DOUBLE),
    FLD(roll_neutral, CFG_DOUBLE),
    FLD(pitch_min, CFG_DOUBLE),
    FLD(pitch_max, CFG_DOUBLE),
    FLD(pitch_neutral, CFG_DOUBLE),
    FLD(yaw_min, CFG_DOUBLE),
    FLD(yaw_max, CFG_DOUBLE),
    FLD(yaw_neutral, CFG_DOUBLE),
    FLD(throttle_min, CFG_DOUBLE),
    FLD(throttle_max, CFG_DOUBLE),
    FLD(throttle_neutral, CFG_DOUBLE),
    FLD(roll_deadzone, CFG_DOUBLE),
    FLD(pitch_deadzone, CFG_DOUBLE),
    FLD(yaw_deadzone, CFG_DOUBLE),
    FLD(throttle_deadzone, CFG_DOUBLE),
    FLD(roll_expo, CFG_DOUBLE),
    FLD(pitch_expo, CFG_DOUBLE),
    FLD(yaw_expo, CFG_DOUBLE),
    FLD(throttle_expo, CFG_DOUBLE),
    FLD(stick_filter_tau, CFG_DOUBLE),
    FLD(throttle_filter_tau, CFG_DOUBLE),
    FLD(arm_button, CFG_INT),
    FLD(takeoff_button, CFG_INT),
    FLD(disarm_button, CFG_INT),
    FLD(arm_pov, CFG_INT),
    FLD(arm_pov_value, CFG_INT),
    FLD(takeoff_pov, CFG_INT),
    FLD(takeoff_pov_value, CFG_INT),
    FLD(disarm_pov, CFG_INT),
    FLD(disarm_pov_value, CFG_INT),
    FLD(usb_timeout_s, CFG_DOUBLE),
    FLD(handover_valid_s, CFG_DOUBLE),
    FLD(max_roll_deg, CFG_DOUBLE),
    FLD(max_pitch_deg, CFG_DOUBLE),
    FLD(max_horizontal_speed, CFG_DOUBLE),
    FLD(speed_blend_band, CFG_DOUBLE),
    FLD(max_ascent_speed, CFG_DOUBLE),
    FLD(max_descent_speed, CFG_DOUBLE),
    FLD(failsafe_descent_speed, CFG_DOUBLE),
    FLD(takeoff_climb_speed, CFG_DOUBLE),
    FLD(takeoff_height, CFG_DOUBLE),
    FLD(max_yaw_rate_deg, CFG_DOUBLE),
    FLD(tilt_slew_deg_s, CFG_DOUBLE),
    FLD(vz_slew, CFG_DOUBLE),
    FLD(yaw_rate_slew_deg, CFG_DOUBLE),
    FLD(brake_speed_enter_hold, CFG_DOUBLE),
    FLD(brake_hold_time, CFG_DOUBLE),
    FLD(landing_enter_height, CFG_DOUBLE),
    FLD(landing_soft_height, CFG_DOUBLE),
    FLD(liftoff_clearance, CFG_DOUBLE),
    FLD(ground_contact_z, CFG_DOUBLE),
    FLD(ground_contact_time, CFG_DOUBLE),
    FLD(contact_vz_max, CFG_DOUBLE),
    FLD(arm_max_tilt_deg, CFG_DOUBLE),
    FLD(arming_time, CFG_DOUBLE),
    FLD(armed_timeout, CFG_DOUBLE),
    FLD(diagnostic_mode, CFG_INT),
    FLD(telemetry_period, CFG_DOUBLE),
    FLD(diagnostic_period, CFG_DOUBLE),
    FLD(hover_thrust, CFG_DOUBLE),
    FLD(idle_thrust, CFG_DOUBLE),
    FLD(motor_command_limit, CFG_DOUBLE),
    FLD(collective_slew, CFG_DOUBLE),
    FLD(degraded_thrust, CFG_DOUBLE),
    FLD(roll_angle_kp, CFG_DOUBLE),
    FLD(roll_rate_kp, CFG_DOUBLE),
    FLD(roll_trim_ki, CFG_DOUBLE),
    FLD(roll_trim_i_limit, CFG_DOUBLE),
    FLD(roll_output_limit, CFG_DOUBLE),
    FLD(pitch_angle_kp, CFG_DOUBLE),
    FLD(pitch_rate_kp, CFG_DOUBLE),
    FLD(pitch_trim_ki, CFG_DOUBLE),
    FLD(pitch_trim_i_limit, CFG_DOUBLE),
    FLD(pitch_output_limit, CFG_DOUBLE),
    FLD(yaw_angle_kp, CFG_DOUBLE),
    FLD(yaw_rate_kp, CFG_DOUBLE),
    FLD(yaw_trim_ki, CFG_DOUBLE),
    FLD(yaw_trim_i_limit, CFG_DOUBLE),
    FLD(yaw_output_limit, CFG_DOUBLE),
    FLD(yaw_hold_rate, CFG_DOUBLE),
    FLD(yaw_lock_time, CFG_DOUBLE),
    FLD(vel_to_tilt, CFG_DOUBLE),
    FLD(vel_tilt_i, CFG_DOUBLE),
    FLD(vel_tilt_i_limit, CFG_DOUBLE),
    FLD(pos_kp, CFG_DOUBLE),
    FLD(pos_ki, CFG_DOUBLE),
    FLD(pos_i_limit, CFG_DOUBLE),
    FLD(alt_kp, CFG_DOUBLE),
    FLD(vz_kp, CFG_DOUBLE),
    FLD(vz_ki, CFG_DOUBLE),
    FLD(vz_i_limit, CFG_DOUBLE),
    FLD(vz_output_limit, CFG_DOUBLE),
    FLD(expected_timestep_ms, CFG_DOUBLE),
};

static void trim_inplace(char *text) {
  char *start;
  char *end;
  size_t len;

  if (!text)
    return;
  start = text;
  while (*start == ' ' || *start == '\t' || *start == '\r' || *start == '\n')
    ++start;
  if (start != text)
    memmove(text, start, strlen(start) + 1);
  len = strlen(text);
  end = text + len;
  while (end > text && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n')) {
    --end;
    *end = '\0';
  }
}

static void set_field(FlightConfig *cfg, const CfgField *field, const char *text) {
  char *addr = ((char *)cfg) + field->offset;
  char *end = NULL;

  if (field->kind == CFG_INT) {
    long value = strtol(text, &end, 10);
    if (end == text)
      return;
    *(int *)addr = (int)value;
  } else {
    double value = strtod(text, &end);
    if (end == text || is_bad(value))
      return;
    *(double *)addr = value;
  }
}

void config_set_defaults(FlightConfig *cfg) {
  if (!cfg)
    return;
  memset(cfg, 0, sizeof(*cfg));

  cfg->roll_axis = -1;
  cfg->pitch_axis = -1;
  cfg->yaw_axis = -1;
  cfg->throttle_axis = -1;
  cfg->roll_min = -32768.0;
  cfg->roll_max = 32767.0;
  cfg->pitch_min = -32768.0;
  cfg->pitch_max = 32767.0;
  cfg->yaw_min = -32768.0;
  cfg->yaw_max = 32767.0;
  cfg->throttle_min = -32768.0;
  cfg->throttle_max = 32767.0;
  cfg->roll_deadzone = 0.08;
  cfg->pitch_deadzone = 0.08;
  cfg->yaw_deadzone = 0.10;
  cfg->throttle_deadzone = 0.12;
  cfg->roll_expo = 0.35;
  cfg->pitch_expo = 0.35;
  cfg->yaw_expo = 0.30;
  cfg->throttle_expo = 0.0;
  cfg->stick_filter_tau = 0.05;
  cfg->throttle_filter_tau = 0.08;
  cfg->arm_button = -1;
  cfg->takeoff_button = -1;
  cfg->disarm_button = -1;
  cfg->arm_pov = -1;
  cfg->arm_pov_value = -1;
  cfg->takeoff_pov = -1;
  cfg->takeoff_pov_value = -1;
  cfg->disarm_pov = -1;
  cfg->disarm_pov_value = -1;
  cfg->usb_timeout_s = 0.20;
  cfg->handover_valid_s = 0.40;

  cfg->max_roll_deg = 25.0;
  cfg->max_pitch_deg = 25.0;
  cfg->max_horizontal_speed = 5.0;
  cfg->speed_blend_band = 0.40;
  /* Vertical speeds are starting guesses. Tune them in the simulator. */
  cfg->max_ascent_speed = 1.0;
  cfg->max_descent_speed = 0.60;
  cfg->failsafe_descent_speed = 0.35;
  cfg->takeoff_climb_speed = 0.50;
  cfg->takeoff_height = 1.0;
  cfg->max_yaw_rate_deg = 40.0;
  cfg->tilt_slew_deg_s = 45.0;
  cfg->vz_slew = 1.2;
  cfg->yaw_rate_slew_deg = 90.0;
  cfg->brake_speed_enter_hold = 0.12;
  cfg->brake_hold_time = 0.25;
  cfg->landing_enter_height = 0.45;
  cfg->landing_soft_height = 0.35;
  cfg->liftoff_clearance = 0.18;
  cfg->ground_contact_z = 0.08;
  cfg->ground_contact_time = 0.30;
  cfg->contact_vz_max = 0.35;
  cfg->arm_max_tilt_deg = 12.0;
  cfg->arming_time = 0.40;
  cfg->armed_timeout = 10.0;
  cfg->diagnostic_mode = 1;
  cfg->telemetry_period = 1.0;
  cfg->diagnostic_period = 0.25;

  /*
   * hover_thrust is the collective used by the Webots R2025a sample
   * mavic2pro.c for this proto. It was not re-measured in this session.
   * Attitude gains are that sample's P and rate terms, rewritten as a cascade:
   * motor ≈ rate_kp * (rate - angle_kp * (setpoint - angle)).
   */
  cfg->hover_thrust = 68.5;
  cfg->idle_thrust = 20.0;
  cfg->motor_command_limit = 160.0;
  cfg->collective_slew = 50.0;
  cfg->degraded_thrust = 64.0;
  cfg->roll_angle_kp = 50.0;
  cfg->roll_rate_kp = 1.0;
  cfg->roll_trim_ki = 1.5;
  cfg->roll_trim_i_limit = 0.35;
  cfg->roll_output_limit = 25.0;
  cfg->pitch_angle_kp = 30.0;
  cfg->pitch_rate_kp = 1.0;
  cfg->pitch_trim_ki = 1.0;
  cfg->pitch_trim_i_limit = 0.35;
  cfg->pitch_output_limit = 20.0;
  cfg->yaw_angle_kp = 4.0;
  cfg->yaw_rate_kp = 0.8;
  cfg->yaw_trim_ki = 0.15;
  cfg->yaw_trim_i_limit = 0.25;
  cfg->yaw_output_limit = 1.6;
  cfg->yaw_hold_rate = 0.08;
  cfg->yaw_lock_time = 0.15;
  cfg->vel_to_tilt = 0.08;
  cfg->vel_tilt_i = 0.02;
  cfg->vel_tilt_i_limit = 0.08;
  cfg->pos_kp = 0.55;
  cfg->pos_ki = 0.08;
  cfg->pos_i_limit = 0.40;
  cfg->alt_kp = 1.0;
  cfg->vz_kp = 4.0;
  cfg->vz_ki = 1.2;
  cfg->vz_i_limit = 2.5;
  cfg->vz_output_limit = 25.0;
  cfg->expected_timestep_ms = 8.0;
}

void config_resolve_path(char *dest, int dest_len) {
  const char *project;

  if (!dest || dest_len < 8)
    return;
  dest[0] = '\0';
  project = wb_robot_get_project_path();
  if (project && project[0]) {
    snprintf(dest, (size_t)dest_len, "%s/controllers/mavic_manual/mavic_manual.cfg", project);
    return;
  }
  snprintf(dest, (size_t)dest_len, "mavic_manual.cfg");
}

/*
 * Webots hands back a UTF-8 project path. The MinGW fopen uses the ANSI
 * code page, so a folder such as "Мавик" makes the open fail and the
 * controller keeps diagnostic defaults: axes unmapped, motors locked.
 */
static FILE *open_utf8(const char *path) {
#ifdef _WIN32
  wchar_t wide[1024];
  int count;
  FILE *file;

  if (!path || path[0] == '\0')
    return NULL;
  count = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, path, -1, wide, 1024);
  if (count > 0) {
    file = _wfopen(wide, L"r");
    if (file)
      return file;
  }
  count = MultiByteToWideChar(CP_ACP, 0, path, -1, wide, 1024);
  if (count > 0) {
    file = _wfopen(wide, L"r");
    if (file)
      return file;
  }
#endif
  return path ? fopen(path, "r") : NULL;
}

static FILE *open_beside_exe(void) {
#ifdef _WIN32
  wchar_t path[1024];
  wchar_t *slash;
  DWORD count = GetModuleFileNameW(NULL, path, 1024);

  if (count == 0 || count >= 1024)
    return NULL;
  slash = wcsrchr(path, L'\\');
  if (!slash)
    slash = wcsrchr(path, L'/');
  if (!slash)
    return NULL;
  if ((size_t)(slash - path) + 20 >= 1024)
    return NULL;
  wcscpy(slash + 1, L"mavic_manual.cfg");
  return _wfopen(path, L"r");
#else
  return NULL;
#endif
}

int config_load(FlightConfig *cfg, const char *path, char *error, int error_len) {
  FILE *file;
  char line[256];
  char opened[512];
  int line_no = 0;
  int applied = 0;

  config_set_defaults(cfg);
  if (error && error_len > 0)
    error[0] = '\0';
  if (!path)
    return 0;

  opened[0] = '\0';
  file = open_utf8(path);
  if (file)
    snprintf(opened, sizeof(opened), "%s", path);
  if (!file) {
    file = open_utf8("mavic_manual.cfg");
    if (file)
      snprintf(opened, sizeof(opened), "%s", "mavic_manual.cfg");
  }
  if (!file) {
    file = open_beside_exe();
    if (file)
      snprintf(opened, sizeof(opened), "%s", "mavic_manual.cfg beside the controller");
  }
  if (!file) {
    if (error && error_len > 0)
      snprintf(error, (size_t)error_len, "config not opened (%s); defaults in use, motors stay locked", path);
    return 0;
  }

  while (fgets(line, (int)sizeof(line), file)) {
    char *hash;
    char *equals;
    size_t i;

    ++line_no;
    hash = strchr(line, '#');
    if (hash)
      *hash = '\0';
    trim_inplace(line);
    if (line[0] == '\0')
      continue;
    equals = strchr(line, '=');
    if (!equals) {
      printf("mavic_manual: ignoring config line %d (no '=')\n", line_no);
      continue;
    }
    *equals = '\0';
    trim_inplace(line);
    trim_inplace(equals + 1);
    for (i = 0; i < sizeof(FIELDS) / sizeof(FIELDS[0]); ++i) {
      if (strcmp(FIELDS[i].key, line) == 0) {
        set_field(cfg, &FIELDS[i], equals + 1);
        ++applied;
        break;
      }
    }
    if (i == sizeof(FIELDS) / sizeof(FIELDS[0]))
      printf("mavic_manual: unknown config key '%s' on line %d\n", line, line_no);
  }
  fclose(file);

  if (cfg->max_horizontal_speed < 0.2)
    cfg->max_horizontal_speed = 0.2;
  if (cfg->motor_command_limit < 1.0)
    cfg->motor_command_limit = 1.0;
  if (cfg->usb_timeout_s < 0.02)
    cfg->usb_timeout_s = 0.02;
  if (cfg->speed_blend_band < 0.05)
    cfg->speed_blend_band = 0.05;
  if (cfg->diagnostic_period < 0.05)
    cfg->diagnostic_period = 0.05;
  if (cfg->telemetry_period < 0.05)
    cfg->telemetry_period = 0.05;

  printf("mavic_manual: loaded %d config values from %s\n", applied, opened);
  return 1;
}
