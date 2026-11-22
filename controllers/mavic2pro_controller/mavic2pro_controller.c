/*
 * mavic2pro_controller.c — autonomous zigzag survey for the DJI Mavic 2 Pro.
 *
 * The mission starts with the simulation: take off, fly the task, return,
 * and land. A mission file uses map metres (X east, Y north, origin at the
 * pad). Without one, mavic2pro_controller.cfg still flies the old rectangle
 * ahead of the launch heading.
 *
 * Device names and the motor mix come from the R2025a Mavic2Pro proto and the
 * stock mavic2pro.c controller. World frame is ENU (X east, Y north, Z up).
 * With yaw 0 the nose points +X and the left side points +Y.
 *
 * Propeller thrustConstants[0] (positive velocity):
 *   front left  +0.00026   command applied as-is
 *   front right -0.00026   command negated
 *   rear left   -0.00026   command negated
 *   rear right  +0.00026   command applied as-is
 *
 * The stock proto has gps, compass, gyro, and inertial unit. This world adds
 * accelerometer and altimeter on bodySlot, at the robot origin.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include <webots/accelerometer.h>
#include <webots/altimeter.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/gyro.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <webots/plugins/robot_window/robot_wwi.h>
#include <webots/robot.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ---------- mission parameters ---------- */
/*
 * The patrol rectangle comes from mavic2pro_controller.cfg. These defaults
 * apply when that file is missing or a value in it is not usable.
 * length runs forward from the launch point along the launch heading.
 * width is centered on that heading, so half of it lies on each side.
 */
static double area_length = 10.0;
static double area_width = 10.0;
static double flight_altitude = 1.0;
static double lane_spacing = 1.0;
#define FLIGHT_SPEED 2.0
#define WAYPOINT_TOLERANCE 0.2

/* Hover thrust from the stock controller: this spin already lifts the model. */
#define K_VERTICAL_THRUST 68.5
#define K_VERTICAL_P 3.0
#define K_VERTICAL_I 1.0
#define K_VERTICAL_D 4.0
#define ALT_I_LIMIT 1.0

#define K_ROLL_P 50.0
#define K_PITCH_P 30.0
#define K_ROLL_I 12.0
#define K_PITCH_I 8.0
#define K_ROLL_D 1.0
#define K_PITCH_D 1.0
#define ATT_I_LIMIT 0.04

/*
 * Velocity error becomes a motor-command offset on top of attitude
 * stabilization. Linear damping needs a steady nose-down tilt to hold
 * FLIGHT_SPEED, which a proportional term alone cannot supply, so a small
 * integral holds that tilt. Roll stays near the stock strafe command of 1:
 * a large roll offset flips this airframe.
 */
#define K_PITCH_VEL 2.0
#define K_ROLL_VEL 0.8
#define K_PITCH_VEL_I 1.8
#define K_ROLL_VEL_I 0.5
#define PITCH_LIMIT 4.0
#define ROLL_LIMIT 1.0
#define PITCH_VEL_I_LIMIT 2.2
#define ROLL_VEL_I_LIMIT 1.2
#define ALIGN_YAW 0.50
#define ALIGN_SPEED 0.35

#define K_YAW_P 1.8
#define K_YAW_I 0.12
#define K_YAW_D 0.40
#define YAW_LIMIT 1.3
#define YAW_I_LIMIT 0.35
#define YAW_SLEW_RATE 1.5

#define K_POS_I 0.35
#define POS_I_LIMIT 0.45
#define BRAKE_ACCEL 0.22
#define HOLD_SPEED 0.6

#define ALT_TOL 0.30
#define VZ_TOL 0.20
#define HOVER_TIME 1.0
#define CLIMB_RATE 0.7
#define DESCENT_RATE 0.55
#define LAND_ALT_BAND 0.35
#define LAND_STABLE_TIME 0.8

#define DEAD_RECKON_MAX 2.0
#define SENSOR_TIMEOUT 3.0
#define ATTITUDE_FAIL_TIME 0.25
#define ALTITUDE_FAIL_TIME 0.50
#define GRAVITY 9.81
#define MOTOR_LIMIT 500.0
#define MAX_WAYPOINTS 512

#define CLAMP(value, low, high) \
  ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)))

typedef enum {
  INITIALIZATION,
  READY,
  TAKEOFF,
  HOVER,
  ZIGZAG_SCAN,
  RETURN_HOME,
  LANDING,
  LANDED,
  WAIT,
  EMERGENCY
} MissionState;

typedef struct {
  double x;
  double y;
} Waypoint;

/* Body-frame velocity, heading, and altitude. The mixer consumes only this. */
typedef struct {
  double v_forward;
  double v_left;
  double yaw_goal;
  double altitude;
} Command;

typedef struct {
  double x;
  double y;
  double vx;
  double vy;
  double vz;
  double alt;
  double gps_x;
  double gps_y;
  double gps_z;
  double roll;
  double pitch;
  double yaw_imu;
  double yaw_compass;
  double yaw;
  double heading_deg;
  double roll_rate;
  double pitch_rate;
  double yaw_rate;
  double vert_acc;
  int gps_ok;
  int position_reliable;
  int altitude_ok;
  int attitude_ok;
  int accel_ok;
  int compass_ok;
} Nav;

static WbDeviceTag gps_dev;
static WbDeviceTag compass_dev;
static WbDeviceTag accel_dev;
static WbDeviceTag gyro_dev;
static WbDeviceTag imu_dev;
static WbDeviceTag alt_dev;
static WbDeviceTag motor_fl;
static WbDeviceTag motor_fr;
static WbDeviceTag motor_rl;
static WbDeviceTag motor_rr;

static MissionState state = INITIALIZATION;
static Command cmd;
static Nav nav;

static Waypoint waypoints[MAX_WAYPOINTS];
static int waypoint_count = 0;
static int waypoint_index = 0;

#define MAP_LIMIT 250.0
#define MAX_SHAPE 64

typedef enum {
  TASK_LEGACY = 0,
  TASK_AREA,
  TASK_ROUTE,
  TASK_OBJECT,
  TASK_WATCH
} TaskKind;

static TaskKind task_kind = TASK_LEGACY;
static int task_ready = 0;
static int task_failed = 0;
static double task_spacing = 5.0;
static double task_width = 0.0;
static double task_radius = 8.0;
static int task_laps = 1;
static int task_repeats = 1;
static int repeat_done = 0;
static int abort_to_wait = 0;
static char pending_start[1200];
static int pending_kind = 0;
static double next_panel_cfg = 0.0;
static double task_cx = 0.0;
static double task_cy = 0.0;
static Waypoint shape[MAX_SHAPE];
static int shape_count = 0;
static int saw_altitude = 0;
static int saw_spacing = 0;
static int saw_width = 0;
static int saw_radius = 0;
static int saw_laps = 0;
static int saw_center = 0;
static int saw_polygon = 0;
static int saw_path = 0;

static int origin_valid = 0;
static int motors_enabled = 0;
static int mission_ok = 0;
static int mission_announced = 0;
static int return_abandoned = 0;
static int land_chase = 1;
static int fatal_failure = 0;
static int gps_loss_reported = 0;
static int alt_backup_reported = 0;
static int fatal_reported = 0;
static int incomplete_reported = 0;

static double home_x = 0.0;
static double home_y = 0.0;
static double home_alt = 0.0;
static double yaw0 = 0.0;
static double flight_z = 0.0;
static double altitude_cmd = 0.0;
static double yaw_slew = 0.0;
static double leg_deadline = 0.0;
static double phase_start = -1.0;
static double hover_timer = 0.0;
static double land_stable = 0.0;
static double bad_attitude_time = 0.0;
static double bad_altitude_time = 0.0;

static double pos_i_forward = 0.0;
static double pos_i_left = 0.0;
static double last_goal_x = 0.0;
static double last_goal_y = 0.0;
static int goal_valid = 0;

static double alt_i = 0.0;
static double roll_i = 0.0;
static double pitch_i = 0.0;
static double yaw_i = 0.0;
static double pitch_vel_i = 0.0;
static double roll_vel_i = 0.0;

static double next_telemetry = 0.0;

static const char *state_name(MissionState mission) {
  switch (mission) {
    case INITIALIZATION:
      return "INITIALIZATION";
    case READY:
      return "READY";
    case TAKEOFF:
      return "TAKEOFF";
    case HOVER:
      return "HOVER";
    case ZIGZAG_SCAN:
      return "ZIGZAG_SCAN";
    case RETURN_HOME:
      return "RETURN_HOME";
    case LANDING:
      return "LANDING";
    case LANDED:
      return "LANDED";
    case WAIT:
      return "WAIT";
    case EMERGENCY:
      return "EMERGENCY";
    default:
      return "EMERGENCY";
  }
}

static double wrap_pi(double angle) {
  while (angle > M_PI)
    angle -= 2.0 * M_PI;
  while (angle <= -M_PI)
    angle += 2.0 * M_PI;
  return angle;
}

static double wrap_360(double degrees) {
  while (degrees < 0.0)
    degrees += 360.0;
  while (degrees >= 360.0)
    degrees -= 360.0;
  return degrees;
}

static int device_ok(WbDeviceTag tag, const char *name) {
  if (tag != 0)
    return 1;
  fprintf(stderr, "mavic2pro_controller: device \"%s\" not found. Mission stopped.\n", name);
  return 0;
}

static void trim_line(char *text) {
  char *start = text;
  char *end;

  while (*start == ' ' || *start == '\t')
    ++start;
  if (start != text)
    memmove(text, start, strlen(start) + 1);
  end = text + strlen(text);
  while (end > text && (end[-1] == ' ' || end[-1] == '\t' || end[-1] == '\r' || end[-1] == '\n'))
    --end;
  *end = '\0';
}

static int parse_number(const char *text, double *out) {
  const char *cursor = text;
  int sign = 1;
  int digits = 0;
  double value = 0.0;
  double place = 0.1;

  if (!cursor || cursor[0] == '\0')
    return 0;
  if (*cursor == '+')
    ++cursor;
  else if (*cursor == '-') {
    sign = -1;
    ++cursor;
  }
  while (*cursor >= '0' && *cursor <= '9') {
    value = value * 10.0 + (double)(*cursor - '0');
    ++digits;
    ++cursor;
  }
  if (*cursor == '.') {
    ++cursor;
    while (*cursor >= '0' && *cursor <= '9') {
      value += (double)(*cursor - '0') * place;
      place *= 0.1;
      ++digits;
      ++cursor;
    }
  }
  if (*cursor != '\0' || digits == 0 || !isfinite(value))
    return 0;
  *out = (double)sign * value;
  return 1;
}

static int apply_zone_key(const char *key, const char *text, int line_no) {
  double value;
  int allow_zero = 0;
  double *slot = NULL;

  if (strcmp(key, "length") == 0)
    slot = &area_length;
  else if (strcmp(key, "width") == 0) {
    slot = &area_width;
    allow_zero = 1;
  } else if (strcmp(key, "altitude") == 0)
    slot = &flight_altitude;
  else if (strcmp(key, "lane_spacing") == 0)
    slot = &lane_spacing;
  else {
    fprintf(stderr, "mavic2pro_controller: unknown key \"%s\" on line %d.\n", key, line_no);
    return 0;
  }
  if (!parse_number(text, &value) || (allow_zero ? value < 0.0 : !(value > 0.0))) {
    fprintf(stderr,
            "mavic2pro_controller: bad value for %s on line %d. The default stays in use.\n",
            key, line_no);
    return 0;
  }
  *slot = value;
  return 1;
}

#ifdef _WIN32
static FILE *open_utf8(const char *path) {
  wchar_t wide[1024];
  int count;

  if (!path || path[0] == '\0')
    return NULL;
  count = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, path, -1, wide, 1024);
  if (count > 0)
    return _wfopen(wide, L"r");
  count = MultiByteToWideChar(CP_ACP, 0, path, -1, wide, 1024);
  if (count > 0)
    return _wfopen(wide, L"r");
  return fopen(path, "r");
}

static FILE *open_beside_exe(void) {
  wchar_t path[1024];
  wchar_t *slash;
  DWORD count = GetModuleFileNameW(NULL, path, 1024);

  if (count == 0 || count >= 1024)
    return NULL;
  slash = wcsrchr(path, L'\\');
  if (!slash)
    slash = wcsrchr(path, L'/');
  if (!slash || (size_t)(slash - path) + 40 >= 1024)
    return NULL;
  wcscpy(slash + 1, L"mavic2pro_controller.cfg");
  return _wfopen(path, L"r");
}
#else
static FILE *open_utf8(const char *path) {
  return path ? fopen(path, "r") : NULL;
}

static FILE *open_beside_exe(void) {
  return NULL;
}
#endif

static void load_zone(void) {
  char path[1024];
  const char *project = wb_robot_get_project_path();
  FILE *file = NULL;
  char line[256];
  int line_no = 0;
  int applied = 0;

  path[0] = '\0';
  if (project && project[0]) {
    snprintf(path, sizeof(path), "%s/controllers/mavic2pro_controller/mavic2pro_controller.cfg", project);
    file = open_utf8(path);
  }
  if (!file)
    file = open_beside_exe();

  if (!file) {
    printf("mavic2pro_controller: zone file not found. Defaults are in use.\n");
  } else {
    while (fgets(line, (int)sizeof(line), file)) {
      char *hash = strchr(line, '#');
      char *equals;

      ++line_no;
      if (hash)
        *hash = '\0';
      trim_line(line);
      if (line[0] == '\0')
        continue;
      equals = strchr(line, '=');
      if (!equals) {
        fprintf(stderr, "mavic2pro_controller: line %d has no '='.\n", line_no);
        continue;
      }
      *equals = '\0';
      trim_line(line);
      trim_line(equals + 1);
      if (apply_zone_key(line, equals + 1, line_no))
        ++applied;
    }
    fclose(file);
    printf("mavic2pro_controller: loaded %d zone values.\n", applied);
  }
  printf("mavic2pro_controller: length %.2f m, width %.2f m, altitude %.2f m, lane spacing %.2f m\n",
         area_length, area_width, flight_altitude, lane_spacing);
  fflush(stdout);
}

static int parse_pair(const char *text, double *x, double *y) {
  const char *cursor = text;
  char token[64];
  int i;

  while (*cursor == ' ' || *cursor == '\t')
    ++cursor;
  i = 0;
  while (*cursor && *cursor != ' ' && *cursor != '\t' && i < 63)
    token[i++] = *cursor++;
  token[i] = '\0';
  if (!parse_number(token, x))
    return 0;
  while (*cursor == ' ' || *cursor == '\t')
    ++cursor;
  i = 0;
  while (*cursor && *cursor != ' ' && *cursor != '\t' && i < 63)
    token[i++] = *cursor++;
  token[i] = '\0';
  if (!parse_number(token, y))
    return 0;
  while (*cursor == ' ' || *cursor == '\t')
    ++cursor;
  return *cursor == '\0';
}

static int inside_map(double x, double y) {
  return fabs(x) <= MAP_LIMIT && fabs(y) <= MAP_LIMIT;
}

static int add_shape_point(double x, double y, int line_no) {
  if (!inside_map(x, y)) {
    fprintf(stderr, "mavic2pro_controller: point on line %d is outside the 500 m map.\n", line_no);
    return 0;
  }
  if (shape_count >= MAX_SHAPE) {
    fprintf(stderr, "mavic2pro_controller: more than %d shape points.\n", MAX_SHAPE);
    return 0;
  }
  shape[shape_count].x = x;
  shape[shape_count].y = y;
  ++shape_count;
  return 1;
}

static const char *task_title(TaskKind kind) {
  switch (kind) {
    case TASK_AREA:
      return "Площадная. Район по углам, галсы внутри, один проход.";
    case TASK_ROUTE:
      return "Маршрутная. Ломаная по дороге и полоса осмотра вдоль неё.";
    case TASK_OBJECT:
      return "Объектная. Круги вокруг одной точки.";
    case TASK_WATCH:
      return "Дозор. Повторный обход замкнутого контура.";
    default:
      return "Прежний прямоугольник перед носом.";
  }
}

static int apply_task_key(const char *key, const char *text, int line_no) {
  double value;

  if (strcmp(key, "mode") == 0) {
    if (strcmp(text, "area") == 0)
      task_kind = TASK_AREA;
    else if (strcmp(text, "route") == 0)
      task_kind = TASK_ROUTE;
    else if (strcmp(text, "object") == 0)
      task_kind = TASK_OBJECT;
    else if (strcmp(text, "watch") == 0)
      task_kind = TASK_WATCH;
    else {
      fprintf(stderr, "mavic2pro_controller: unknown mode \"%s\" on line %d.\n", text, line_no);
      return 0;
    }
    return 1;
  }
  if (strcmp(key, "altitude") == 0) {
    if (!parse_number(text, &value) || !(value > 0.0) || value > 80.0) {
      fprintf(stderr, "mavic2pro_controller: bad altitude on line %d.\n", line_no);
      return 0;
    }
    flight_altitude = value;
    saw_altitude = 1;
    return 1;
  }
  if (strcmp(key, "lane_spacing") == 0) {
    if (!parse_number(text, &value) || !(value > 0.0)) {
      fprintf(stderr, "mavic2pro_controller: bad lane_spacing on line %d.\n", line_no);
      return 0;
    }
    task_spacing = value;
    saw_spacing = 1;
    return 1;
  }
  if (strcmp(key, "width") == 0) {
    if (!parse_number(text, &value) || value < 0.0) {
      fprintf(stderr, "mavic2pro_controller: bad width on line %d.\n", line_no);
      return 0;
    }
    task_width = value;
    saw_width = 1;
    return 1;
  }
  if (strcmp(key, "radius") == 0) {
    if (!parse_number(text, &value) || !(value > 0.0)) {
      fprintf(stderr, "mavic2pro_controller: bad radius on line %d.\n", line_no);
      return 0;
    }
    task_radius = value;
    saw_radius = 1;
    return 1;
  }
  if (strcmp(key, "laps") == 0) {
    if (!parse_number(text, &value) || value < 1.0 || value > 20.0) {
      fprintf(stderr, "mavic2pro_controller: bad laps on line %d.\n", line_no);
      return 0;
    }
    task_laps = (int)(value + 0.5);
    saw_laps = 1;
    return 1;
  }
  if (strcmp(key, "center") == 0) {
    if (!parse_pair(text, &task_cx, &task_cy) || !inside_map(task_cx, task_cy)) {
      fprintf(stderr, "mavic2pro_controller: bad center on line %d.\n", line_no);
      return 0;
    }
    saw_center = 1;
    return 1;
  }
  if (strcmp(key, "polygon") == 0) {
    saw_polygon = 1;
    return 1;
  }
  if (strcmp(key, "path") == 0) {
    saw_path = 1;
    return 1;
  }
  fprintf(stderr, "mavic2pro_controller: unknown key \"%s\" on line %d.\n", key, line_no);
  return 0;
}

static int key_unused(const char *key) {
  fprintf(stderr, "mavic2pro_controller: \"%s\" is not used by this mode.\n", key);
  return 0;
}

static int task_shape_ok(const char *selected) {
  int i;
  const char *mode_name = NULL;

  if (task_kind == TASK_AREA)
    mode_name = "area";
  else if (task_kind == TASK_ROUTE)
    mode_name = "route";
  else if (task_kind == TASK_OBJECT)
    mode_name = "object";
  else if (task_kind == TASK_WATCH)
    mode_name = "watch";
  if (!mode_name) {
    fprintf(stderr, "mavic2pro_controller: the task file needs mode = %s.\n", selected);
    return 0;
  }
  if (strcmp(mode_name, selected) != 0) {
    fprintf(stderr, "mavic2pro_controller: mode is %s, but the selected file is %s.\n", mode_name, selected);
    return 0;
  }
  if (!saw_altitude || !(flight_altitude > 0.0)) {
    fprintf(stderr, "mavic2pro_controller: altitude is required.\n");
    return 0;
  }
  if (task_kind == TASK_AREA) {
    if (saw_width || saw_radius || saw_laps || saw_center || saw_path)
      return key_unused(saw_path ? "path" : (saw_width ? "width" : (saw_radius ? "radius" : (saw_laps ? "laps" : "center"))));
    if (!saw_polygon || !saw_spacing || shape_count < 3) {
      fprintf(stderr, "mavic2pro_controller: area needs polygon and lane_spacing.\n");
      return 0;
    }
  } else if (task_kind == TASK_ROUTE) {
    if (saw_polygon || saw_radius || saw_laps || saw_center)
      return key_unused(saw_polygon ? "polygon" : (saw_radius ? "radius" : (saw_laps ? "laps" : "center")));
    if (!saw_path || !saw_width || !saw_spacing || shape_count < 2) {
      fprintf(stderr, "mavic2pro_controller: route needs path, width and lane_spacing.\n");
      return 0;
    }
  } else if (task_kind == TASK_OBJECT) {
    if (saw_polygon || saw_path || saw_width || saw_spacing)
      return key_unused(saw_polygon ? "polygon" : (saw_path ? "path" : (saw_width ? "width" : "lane_spacing")));
    if (!saw_center || !saw_radius || !saw_laps || !(task_radius > 0.0)) {
      fprintf(stderr, "mavic2pro_controller: object needs center, radius and laps.\n");
      return 0;
    }
    if (!inside_map(task_cx - task_radius, task_cy) || !inside_map(task_cx + task_radius, task_cy) ||
        !inside_map(task_cx, task_cy - task_radius) || !inside_map(task_cx, task_cy + task_radius)) {
      fprintf(stderr, "mavic2pro_controller: the orbit does not fit on the map.\n");
      return 0;
    }
    return 1;
  } else {
    if (saw_path || saw_width || saw_spacing || saw_radius || saw_center)
      return key_unused(saw_path ? "path" : (saw_width ? "width" : (saw_spacing ? "lane_spacing" : (saw_radius ? "radius" : "center"))));
    if (!saw_polygon || !saw_laps || shape_count < 3) {
      fprintf(stderr, "mavic2pro_controller: watch needs polygon and laps.\n");
      return 0;
    }
  }
  for (i = 0; i < shape_count; ++i) {
    if (!inside_map(shape[i].x, shape[i].y))
      return 0;
  }
  return 1;
}

static void print_task_values(void) {
  if (task_kind == TASK_AREA)
    printf("Параметры: высота %.2f м, шаг %.2f м, углов %d.\n", flight_altitude, task_spacing, shape_count);
  else if (task_kind == TASK_ROUTE)
    printf("Параметры: высота %.2f м, ширина полосы %.2f м, шаг %.2f м, точек линии %d.\n",
           flight_altitude, task_width, task_spacing, shape_count);
  else if (task_kind == TASK_OBJECT)
    printf("Параметры: высота %.2f м, центр %.2f %.2f, радиус %.2f м, кругов %d.\n",
           flight_altitude, task_cx, task_cy, task_radius, task_laps);
  else if (task_kind == TASK_WATCH)
    printf("Параметры: высота %.2f м, обходов %d, вершин %d.\n", flight_altitude, task_laps, shape_count);
  printf("Повторов задания: %d.\n", task_repeats);
  fflush(stdout);
}

/* Returns 1 when mission.txt selected a task, including a task that failed to load. */
static int load_task(void) {
  char path[1024];
  char mission_path[1024];
  char name[32];
  char line[256];
  const char *project = wb_robot_get_project_path();
  const char *file_name = NULL;
  FILE *file;
  int line_no = 0;
  int reading_points = 0;
  int points_ok = 1;

  name[0] = '\0';
  if (!project || !project[0])
    return 0;
  snprintf(path, sizeof(path), "%s/controllers/mavic2pro_controller/mission.txt", project);
  file = open_utf8(path);
  if (!file)
    return 0;
  if (!fgets(name, (int)sizeof(name), file)) {
    fclose(file);
    fprintf(stderr, "mavic2pro_controller: mission.txt is empty.\n");
    task_failed = 1;
    return 1;
  }
  fclose(file);
  trim_line(name);
  if (strcmp(name, "area") == 0)
    file_name = "area.mission";
  else if (strcmp(name, "route") == 0)
    file_name = "route.mission";
  else if (strcmp(name, "object") == 0)
    file_name = "object.mission";
  else if (strcmp(name, "watch") == 0)
    file_name = "watch.mission";
  else {
    fprintf(stderr, "mavic2pro_controller: unknown task \"%s\".\n", name);
    task_failed = 1;
    return 1;
  }

  snprintf(mission_path, sizeof(mission_path), "%s/controllers/mavic2pro_controller/missions/%s", project, file_name);
  file = open_utf8(mission_path);
  if (!file) {
    fprintf(stderr, "mavic2pro_controller: task file not found: %s\n", file_name);
    task_failed = 1;
    return 1;
  }

  while (fgets(line, (int)sizeof(line), file)) {
    char *hash = strchr(line, '#');
    char *equals;
    double x;
    double y;

    ++line_no;
    if (hash)
      *hash = '\0';
    trim_line(line);
    if (line[0] == '\0') {
      reading_points = 0;
      continue;
    }
    equals = strchr(line, '=');
    if (equals) {
      reading_points = 0;
      *equals = '\0';
      trim_line(line);
      trim_line(equals + 1);
      if (!apply_task_key(line, equals + 1, line_no))
        points_ok = 0;
      if (strcmp(line, "polygon") == 0 || strcmp(line, "path") == 0)
        reading_points = 1;
      continue;
    }
    if (reading_points && parse_pair(line, &x, &y)) {
      if (!add_shape_point(x, y, line_no))
        points_ok = 0;
      continue;
    }
    fprintf(stderr, "mavic2pro_controller: cannot read line %d.\n", line_no);
    points_ok = 0;
  }
  fclose(file);

  if (shape_count >= 2 && hypot(shape[shape_count - 1].x - shape[0].x, shape[shape_count - 1].y - shape[0].y) < 0.05)
    --shape_count;
  if (!points_ok || !task_shape_ok(name)) {
    task_failed = 1;
    fprintf(stderr, "mavic2pro_controller: task was not accepted.\n");
    return 1;
  }
  task_ready = 1;
  printf("Режим: %s\n", task_title(task_kind));
  print_task_values();
  return 1;
}

static int parameters_ok(void) {
  int intervals;
  int lanes;
  int count;

  if (task_failed) {
    fprintf(stderr, "mavic2pro_controller: mission stopped because the task file is not usable.\n");
    return 0;
  }
  if (task_ready)
    return 1;

  if (!(area_length > 0.0) || !(flight_altitude > 0.0) || !(FLIGHT_SPEED > 0.0) ||
      !(WAYPOINT_TOLERANCE > 0.0) || !(lane_spacing > 0.0) || area_width < 0.0) {
    fprintf(stderr,
            "mavic2pro_controller: mission parameters must be positive "
            "(width may be zero). Mission stopped.\n");
    return 0;
  }

  if (area_width == 0.0) {
    lanes = 1;
  } else {
    intervals = (int)ceil(area_width / lane_spacing - 1e-6);
    if (intervals < 1)
      intervals = 1;
    lanes = intervals + 1;
  }
  count = lanes * 2;
  if (count > MAX_WAYPOINTS) {
    fprintf(stderr,
            "mavic2pro_controller: %d waypoints exceed the limit of %d. "
            "Increase lane_spacing or reduce width. Mission stopped.\n",
            count, MAX_WAYPOINTS);
    return 0;
  }
  return 1;
}

/* Upright compass in ENU: 0 deg is north, 90 deg is east, clockwise. */
static double compass_heading_deg(const double *north) {
  double degrees = atan2(north[1], north[0]) * 180.0 / M_PI;
  return wrap_360(degrees);
}

/* Same angle as the inertial-unit yaw: 0 faces +X, positive toward +Y. */
static double compass_yaw_rad(const double *north) {
  return atan2(north[0], north[1]);
}

static double heading_from_yaw(double yaw) {
  return wrap_360((M_PI / 2.0 - yaw) * 180.0 / M_PI);
}

/*
 * Inertial-unit order is Rz(yaw) * Ry(pitch) * Rx(roll).
 * The accelerometer reads specific force, about +g upward at rest.
 */
static void body_to_world(double roll, double pitch, double yaw, double bx, double by, double bz,
                          double *wx, double *wy, double *wz) {
  const double cr = cos(roll);
  const double sr = sin(roll);
  const double cp = cos(pitch);
  const double sp = sin(pitch);
  const double cy = cos(yaw);
  const double sy = sin(yaw);

  *wx = cy * cp * bx + (cy * sp * sr - sy * cr) * by + (cy * sp * cr + sy * sr) * bz;
  *wy = sy * cp * bx + (sy * sp * sr + cy * cr) * by + (sy * sp * cr - cy * sr) * bz;
  *wz = -sp * bx + cp * sr * by + cp * cr * bz;
}

static void world_to_body(double yaw, double dx, double dy, double *forward, double *left) {
  const double c = cos(yaw);
  const double s = sin(yaw);
  *forward = c * dx + s * dy;
  *left = -s * dx + c * dy;
}

static void local_to_world(double lx, double ly, double *wx, double *wy) {
  const double forward_x = cos(yaw0);
  const double forward_y = sin(yaw0);
  const double left_x = -sin(yaw0);
  const double left_y = cos(yaw0);
  *wx = home_x + lx * forward_x + ly * left_x;
  *wy = home_y + lx * forward_y + ly * left_y;
}

static int push_waypoint(double x, double y) {
  if (waypoint_count >= MAX_WAYPOINTS) {
    fprintf(stderr, "mavic2pro_controller: waypoint list overflow. Mission stopped.\n");
    return 0;
  }
  waypoints[waypoint_count].x = x;
  waypoints[waypoint_count].y = y;
  waypoint_count++;
  return 1;
}

/*
 * Lanes run along the initial heading and include both side edges.
 * Spacing is area_width / ceil(area_width / lane_spacing), so it never
 * exceeds lane_spacing. A zero width is a single centerline.
 * Lane 0 starts on the left edge. Even lanes fly away, odd lanes fly back.
 * Connectors stay on the near and far edges.
 */
static int build_waypoints(void) {
  int intervals = 1;
  int lanes;
  int i;
  double spacing = 0.0;

  waypoint_count = 0;
  if (area_width > 0.0) {
    intervals = (int)ceil(area_width / lane_spacing - 1e-6);
    if (intervals < 1)
      intervals = 1;
    spacing = area_width / (double)intervals;
    lanes = intervals + 1;
  } else {
    lanes = 1;
  }

  for (i = 0; i < lanes; ++i) {
    const double y_local = area_width * 0.5 - (double)i * spacing;
    const int outbound = (i % 2) == 0;
    const double x0 = outbound ? 0.0 : area_length;
    const double x1 = outbound ? area_length : 0.0;
    double wx;
    double wy;

    local_to_world(x0, y_local, &wx, &wy);
    if (!push_waypoint(wx, wy))
      return 0;
    local_to_world(x1, y_local, &wx, &wy);
    if (!push_waypoint(wx, wy))
      return 0;
  }
  return waypoint_count > 0;
}

static void sort_xs(double *xs, int count) {
  int i;
  int j;

  for (i = 1; i < count; ++i) {
    double value = xs[i];
    j = i;
    while (j > 0 && xs[j - 1] > value) {
      xs[j] = xs[j - 1];
      --j;
    }
    xs[j] = value;
  }
}

static int horizontal_hit(double y, Waypoint from, Waypoint to, double *x) {
  double dy = to.y - from.y;
  double t;

  if (fabs(dy) < 1e-9)
    return 0;
  if (from.y < to.y) {
    if (y < from.y || y >= to.y)
      return 0;
  } else if (y < to.y || y >= from.y) {
    return 0;
  }
  t = (y - from.y) / dy;
  *x = from.x + t * (to.x - from.x);
  return 1;
}

static int build_area_task(void) {
  double y_min = shape[0].y;
  double y_max = shape[0].y;
  double y;
  int forward = 1;
  int i;

  for (i = 1; i < shape_count; ++i) {
    if (shape[i].y < y_min)
      y_min = shape[i].y;
    if (shape[i].y > y_max)
      y_max = shape[i].y;
  }
  if (y_max - y_min < 0.5) {
    fprintf(stderr, "mavic2pro_controller: the area has no height.\n");
    return 0;
  }

  y = y_min + task_spacing * 0.5;
  if (y >= y_max)
    y = (y_min + y_max) * 0.5;
  while (y < y_max - 1e-3) {
    double xs[MAX_SHAPE];
    int hits = 0;

    for (i = 0; i < shape_count; ++i) {
      Waypoint next = shape[(i + 1) % shape_count];
      double x;
      if (horizontal_hit(y, shape[i], next, &x) && hits < MAX_SHAPE)
        xs[hits++] = x;
    }
    sort_xs(xs, hits);
    if (hits >= 2) {
      double left = xs[0];
      double right = xs[hits - 1];
      if (right - left > 0.5) {
        double x0 = forward ? left : right;
        double x1 = forward ? right : left;
        if (!push_waypoint(x0, y) || !push_waypoint(x1, y))
          return 0;
        forward = !forward;
      }
    }
    if (task_spacing >= y_max - y_min)
      break;
    y += task_spacing;
  }
  return waypoint_count > 0;
}

static int left_normal(double ax, double ay, double bx, double by, double *nx, double *ny) {
  double dx = bx - ax;
  double dy = by - ay;
  double length = hypot(dx, dy);

  if (length < 1e-6)
    return 0;
  *nx = -dy / length;
  *ny = dx / length;
  return 1;
}

static int build_route_task(void) {
  int lanes = 1;
  int lane;
  int forward = 1;

  if (task_width >= 2.0 * task_spacing && task_spacing > 0.0) {
    lanes = (int)floor(task_width / task_spacing + 1e-6);
    if (lanes < 2)
      lanes = 2;
  }
  for (lane = 0; lane < lanes; ++lane) {
    double offset = 0.0;
    double span = (double)(lanes - 1) * task_spacing;
    Waypoint shifted[MAX_SHAPE];
    int i;
    int begin;
    int step;
    int end;

    if (lanes > 1)
      offset = -span * 0.5 + (double)lane * task_spacing;
    for (i = 0; i < shape_count; ++i) {
      double nx = 0.0;
      double ny = 0.0;
      double ax;
      double ay;
      double bx;
      double by;
      int normals = 0;

      if (i > 0 && left_normal(shape[i - 1].x, shape[i - 1].y, shape[i].x, shape[i].y, &ax, &ay)) {
        nx += ax;
        ny += ay;
        ++normals;
      }
      if (i + 1 < shape_count &&
          left_normal(shape[i].x, shape[i].y, shape[i + 1].x, shape[i + 1].y, &bx, &by)) {
        nx += bx;
        ny += by;
        ++normals;
      }
      if (normals > 0) {
        double length = hypot(nx, ny);
        if (length > 1e-6) {
          nx /= length;
          ny /= length;
        }
      }
      shifted[i].x = shape[i].x + nx * offset;
      shifted[i].y = shape[i].y + ny * offset;
      if (!inside_map(shifted[i].x, shifted[i].y)) {
        fprintf(stderr, "mavic2pro_controller: the route corridor leaves the map.\n");
        return 0;
      }
    }
    begin = forward ? 0 : shape_count - 1;
    end = forward ? shape_count : -1;
    step = forward ? 1 : -1;
    for (i = begin; i != end; i += step) {
      if (!push_waypoint(shifted[i].x, shifted[i].y))
        return 0;
    }
    forward = !forward;
  }
  return waypoint_count > 0;
}

static int build_object_task(void) {
  int steps = (int)ceil(2.0 * M_PI * task_radius / 3.0);
  double angle0 = atan2(home_y - task_cy, home_x - task_cx);
  int lap;
  int i;

  if (steps < 8)
    steps = 8;
  if (steps > 36)
    steps = 36;
  for (lap = 0; lap < task_laps; ++lap) {
    for (i = 0; i <= steps; ++i) {
      double angle = angle0 + (double)i * (2.0 * M_PI / (double)steps);
      if (!push_waypoint(task_cx + task_radius * cos(angle), task_cy + task_radius * sin(angle)))
        return 0;
    }
  }
  return waypoint_count > 0;
}

static int build_watch_task(void) {
  int lap;
  int i;

  for (lap = 0; lap < task_laps; ++lap) {
    for (i = 0; i < shape_count; ++i) {
      if (!push_waypoint(shape[i].x, shape[i].y))
        return 0;
    }
    if (!push_waypoint(shape[0].x, shape[0].y))
      return 0;
  }
  return waypoint_count > 0;
}

static int build_task_waypoints(void) {
  int built = 0;

  waypoint_count = 0;
  if (task_kind == TASK_AREA)
    built = build_area_task();
  else if (task_kind == TASK_ROUTE)
    built = build_route_task();
  else if (task_kind == TASK_OBJECT)
    built = build_object_task();
  else if (task_kind == TASK_WATCH)
    built = build_watch_task();
  if (!built) {
    fprintf(stderr, "mavic2pro_controller: the task produced no waypoints.\n");
    return 0;
  }
  printf("Точек маршрута: %d. Высота: %.2f м.\n", waypoint_count, flight_altitude);
  fflush(stdout);
  return 1;
}

static void motors_off(void) {
  wb_motor_set_velocity(motor_fl, 0.0);
  wb_motor_set_velocity(motor_fr, 0.0);
  wb_motor_set_velocity(motor_rl, 0.0);
  wb_motor_set_velocity(motor_rr, 0.0);
}

/* Hover: no horizontal velocity. Altitude and heading are still tracked. */
static void command_hover(double yaw, double altitude) {
  cmd.v_forward = 0.0;
  cmd.v_left = 0.0;
  cmd.yaw_goal = yaw;
  cmd.altitude = altitude;
}

/*
 * Signed body velocity is the abstract move command:
 *   forward  > 0 nose, < 0 tail
 *   left     > 0 left, < 0 right
 * A different yaw rotates the nose toward the direction of travel.
 * altitude above or below the measured height climbs or descends.
 */
static void command_move(double forward, double left, double yaw, double altitude) {
  cmd.v_forward = forward;
  cmd.v_left = left;
  cmd.yaw_goal = yaw;
  cmd.altitude = altitude;
}

static void reset_position_integral(void) {
  pos_i_forward = 0.0;
  pos_i_left = 0.0;
  pitch_vel_i = 0.0;
  roll_vel_i = 0.0;
}

/* Position loop: desired speed follows a brake curve, then a small integral. */
static void velocity_from_position(double goal_x, double goal_y, double speed_cap, double dt,
                                   double *forward, double *left) {
  const double dx = goal_x - nav.x;
  const double dy = goal_y - nav.y;
  const double dist = hypot(dx, dy);
  const double d_stop = fmax(0.0, dist - 0.5 * WAYPOINT_TOLERANCE);
  const double v_brake = sqrt(2.0 * BRAKE_ACCEL * d_stop);
  const double v_mag = fmin(speed_cap, v_brake);
  double err_forward;
  double err_left;

  if (!goal_valid || hypot(goal_x - last_goal_x, goal_y - last_goal_y) > 0.5) {
    reset_position_integral();
    last_goal_x = goal_x;
    last_goal_y = goal_y;
    goal_valid = 1;
  }

  world_to_body(nav.yaw_imu, dx, dy, &err_forward, &err_left);
  if (dist > 1e-6) {
    *forward = v_mag * err_forward / dist;
    *left = v_mag * err_left / dist;
  } else {
    *forward = 0.0;
    *left = 0.0;
  }

  if (dist < 1.5) {
    pos_i_forward = CLAMP(pos_i_forward + err_forward * dt, -POS_I_LIMIT, POS_I_LIMIT);
    pos_i_left = CLAMP(pos_i_left + err_left * dt, -POS_I_LIMIT, POS_I_LIMIT);
  } else {
    pos_i_forward *= 0.95;
    pos_i_left *= 0.95;
  }

  *forward = CLAMP(*forward + K_POS_I * pos_i_forward, -speed_cap, speed_cap);
  *left = CLAMP(*left + K_POS_I * pos_i_left, -speed_cap, speed_cap);
}

static void hold_position(double goal_x, double goal_y, double altitude, double yaw, double speed_cap,
                          double dt) {
  double forward;
  double left;
  velocity_from_position(goal_x, goal_y, speed_cap, dt, &forward, &left);
  command_move(forward, left, yaw, altitude);
}

/* Point the nose along the leg, and creep until that turn is underway. */
static void guide_to(double goal_x, double goal_y, double altitude, double speed_cap, double dt) {
  const double dx = goal_x - nav.x;
  const double dy = goal_y - nav.y;
  const double dist = hypot(dx, dy);
  double yaw = yaw_slew;
  double cap = speed_cap;

  if (dist >= WAYPOINT_TOLERANCE) {
    yaw = atan2(dy, dx);
    if (fabs(wrap_pi(yaw - nav.yaw)) > ALIGN_YAW)
      cap = fmin(cap, ALIGN_SPEED);
  }
  hold_position(goal_x, goal_y, altitude, yaw, cap, dt);
}

static void arm_timer(double time, double goal_x, double goal_y) {
  const double dist = hypot(goal_x - nav.x, goal_y - nav.y);
  leg_deadline = time + fmax(12.0, 4.0 * dist / FLIGHT_SPEED + 4.0);
}

static int waypoint_reached(double goal_x, double goal_y) {
  const double dist = hypot(goal_x - nav.x, goal_y - nav.y);
  const double speed = hypot(nav.vx, nav.vy);
  return dist < WAYPOINT_TOLERANCE && speed < 0.55;
}

static void altitude_ramp(double target, double rate, double dt) {
  const double step = rate * dt;
  if (altitude_cmd < target)
    altitude_cmd = fmin(target, altitude_cmd + step);
  else
    altitude_cmd = fmax(target, altitude_cmd - step);
}

static void update_navigation(double dt) {
  const double *gps_values = wb_gps_get_values(gps_dev);
  const double *north = wb_compass_get_values(compass_dev);
  const double *accel = wb_accelerometer_get_values(accel_dev);
  const double *gyro = wb_gyro_get_values(gyro_dev);
  const double *rpy = wb_inertial_unit_get_roll_pitch_yaw(imu_dev);
  const double altitude = wb_altimeter_get_value(alt_dev);

  static int gps_was_ok = 0;
  static int alt_ready = 0;
  static double prev_gps_x = 0.0;
  static double prev_gps_y = 0.0;
  static double prev_alt = 0.0;
  static double vx_f = 0.0;
  static double vy_f = 0.0;
  static double vz_f = 0.0;
  static double vert_f = 0.0;
  static double gps_lost = 0.0;
  static double last_gps_z = 0.0;

  double ax_w = 0.0;
  double ay_w = 0.0;
  double az_w = 0.0;

  nav.gps_ok = gps_values && isfinite(gps_values[0]) && isfinite(gps_values[1]) && isfinite(gps_values[2]);
  nav.compass_ok = north && isfinite(north[0]) && isfinite(north[1]) && hypot(north[0], north[1]) > 0.5;
  nav.accel_ok = accel && isfinite(accel[0]) && isfinite(accel[1]) && isfinite(accel[2]);
  nav.attitude_ok = rpy && gyro && isfinite(rpy[0]) && isfinite(rpy[1]) && isfinite(rpy[2]) &&
                    isfinite(gyro[0]) && isfinite(gyro[1]) && isfinite(gyro[2]);
  nav.altitude_ok = isfinite(altitude) || nav.gps_ok;

  if (nav.attitude_ok) {
    nav.roll = rpy[0];
    nav.pitch = rpy[1];
    nav.yaw_imu = rpy[2];
    nav.roll_rate = gyro[0];
    nav.pitch_rate = gyro[1];
    nav.yaw_rate = gyro[2];
    bad_attitude_time = 0.0;
  } else {
    bad_attitude_time += dt;
  }

  /*
   * The nose-pointing loop uses inertial yaw, which is the same angle the
   * attitude mixer stabilizes. The compass still supplies the displayed
   * heading and the scan direction captured at launch. A banked compass
   * reading is not a usable yaw feedback.
   */
  if (nav.attitude_ok)
    nav.yaw = nav.yaw_imu;
  if (nav.compass_ok) {
    nav.yaw_compass = compass_yaw_rad(north);
    nav.heading_deg = compass_heading_deg(north);
  } else if (nav.attitude_ok) {
    nav.heading_deg = heading_from_yaw(nav.yaw_imu);
  }

  if (nav.accel_ok && nav.attitude_ok) {
    static int vert_ready = 0;
    const double accel_mag = hypot(hypot(accel[0], accel[1]), accel[2]);
    /* A near-zero sample is the sensor not ready yet, not free fall. */
    if (accel_mag > 2.0)
      body_to_world(nav.roll, nav.pitch, nav.yaw_imu, accel[0], accel[1], accel[2], &ax_w, &ay_w, &az_w);
    else
      nav.accel_ok = 0;
    if (nav.accel_ok) {
      if (!vert_ready) {
        vert_f = az_w - GRAVITY;
        vert_ready = 1;
      } else {
        vert_f += 0.12 * ((az_w - GRAVITY) - vert_f);
      }
      nav.vert_acc = vert_f;
    }
  }

  if (isfinite(altitude)) {
    nav.alt = altitude;
    if (!alt_ready) {
      prev_alt = altitude;
      alt_ready = 1;
    }
  } else if (nav.gps_ok) {
    nav.alt = gps_values[2];
    if (!alt_ready) {
      prev_alt = nav.alt;
      alt_ready = 1;
    }
    if (!alt_backup_reported) {
      fprintf(stderr, "mavic2pro_controller: altimeter unavailable. Altitude is taken from GPS Z.\n");
      alt_backup_reported = 1;
    }
  }

  if (nav.altitude_ok && alt_ready) {
    const double vz_raw = CLAMP((nav.alt - prev_alt) / dt, -8.0, 8.0);
    vz_f += 0.35 * (vz_raw - vz_f);
    nav.vz = vz_f;
    prev_alt = nav.alt;
    bad_altitude_time = 0.0;
  } else if (!nav.altitude_ok) {
    bad_altitude_time += dt;
    nav.vz = 0.0;
  } else {
    prev_alt = nav.alt;
    bad_altitude_time = 0.0;
  }

  if (nav.gps_ok) {
    nav.gps_x = gps_values[0];
    nav.gps_y = gps_values[1];
    nav.gps_z = gps_values[2];
    last_gps_z = nav.gps_z;
    if (gps_was_ok && gps_lost == 0.0) {
      const double vx_raw = (nav.gps_x - prev_gps_x) / dt;
      const double vy_raw = (nav.gps_y - prev_gps_y) / dt;
      vx_f += 0.35 * (vx_raw - vx_f);
      vy_f += 0.35 * (vy_raw - vy_f);
    }
    prev_gps_x = nav.gps_x;
    prev_gps_y = nav.gps_y;
    nav.x = nav.gps_x;
    nav.y = nav.gps_y;
    nav.vx = vx_f;
    nav.vy = vy_f;
    gps_lost = 0.0;
    gps_was_ok = 1;
    gps_loss_reported = 0;
    nav.position_reliable = 1;
  } else {
    gps_lost += dt;
    nav.gps_x = nav.x;
    nav.gps_y = nav.y;
    nav.gps_z = last_gps_z;
    if (gps_lost <= DEAD_RECKON_MAX && nav.accel_ok && nav.attitude_ok && gps_was_ok) {
      /* Short gap only. Gyro and the inertial unit keep the attitude used here. */
      vx_f += ax_w * dt;
      vy_f += ay_w * dt;
      nav.x += vx_f * dt;
      nav.y += vy_f * dt;
      nav.vx = vx_f;
      nav.vy = vy_f;
      nav.position_reliable = 1;
    } else {
      nav.vx = vx_f;
      nav.vy = vy_f;
      nav.position_reliable = 0;
      if (!gps_loss_reported && gps_lost > DEAD_RECKON_MAX) {
        fprintf(stderr,
                "mavic2pro_controller: GPS unavailable for %.1f s. "
                "Dead reckoning stopped. Position is not reliable enough to continue the scan "
                "or to promise a return to the launch point.\n",
                DEAD_RECKON_MAX);
        gps_loss_reported = 1;
      }
    }
  }
}

/*
 * negate is set for pitch: a positive forward-speed error must lower the nose.
 * Back-calculation keeps the integrator at the clamp, and it leaks once the
 * drone is hovering so a cruise tilt is not carried into the landing.
 */
static double velocity_offset(double *integ, double error, double command, double k_p, double k_i,
                              double limit, double i_limit, double dt, int negate) {
  double effort;
  double offset;

  if (fabs(command) < 0.15 && fabs(command - error) < 0.30)
    *integ *= 0.90;
  /* Drop a cruise tilt as soon as the speed command falls below the actual speed. */
  else if (*integ * error < 0.0)
    *integ *= (1.0 - 6.0 * dt);

  *integ = CLAMP(*integ + error * dt, -i_limit, i_limit);
  effort = k_p * error + k_i * (*integ);
  offset = negate ? -effort : effort;
  if (offset > limit)
    offset = limit;
  else if (offset < -limit)
    offset = -limit;
  if (k_i > 1e-9) {
    effort = negate ? -offset : offset;
    *integ = CLAMP((effort - k_p * error) / k_i, -i_limit, i_limit);
  }
  return offset;
}

static void apply_control(double dt) {
  double alt_error;
  double vertical_input;
  double v_forward;
  double v_left;
  double pitch_disturbance;
  double roll_disturbance;
  double yaw_error;
  double yaw_step;
  double yaw_input;
  double roll_input;
  double pitch_input;
  double front_left;
  double front_right;
  double rear_left;
  double rear_right;
  const double max_yaw_step = YAW_SLEW_RATE * dt;

  if (!nav.attitude_ok || !nav.altitude_ok)
    return;

  alt_error = cmd.altitude - nav.alt;
  if (fabs(alt_error) < 0.5)
    alt_i = CLAMP(alt_i + alt_error * dt, -ALT_I_LIMIT, ALT_I_LIMIT);
  else
    alt_i *= 0.98;
  vertical_input = K_VERTICAL_P * CLAMP(alt_error, -1.0, 1.0) + K_VERTICAL_I * alt_i - K_VERTICAL_D * nav.vz;

  world_to_body(nav.yaw_imu, nav.vx, nav.vy, &v_forward, &v_left);
  pitch_disturbance = velocity_offset(&pitch_vel_i, cmd.v_forward - v_forward, cmd.v_forward, K_PITCH_VEL,
                                      K_PITCH_VEL_I, PITCH_LIMIT, PITCH_VEL_I_LIMIT, dt, 1);
  roll_disturbance = velocity_offset(&roll_vel_i, cmd.v_left - v_left, cmd.v_left, K_ROLL_VEL, K_ROLL_VEL_I,
                                     ROLL_LIMIT, ROLL_VEL_I_LIMIT, dt, 0);

  yaw_step = wrap_pi(cmd.yaw_goal - yaw_slew);
  yaw_step = CLAMP(yaw_step, -max_yaw_step, max_yaw_step);
  yaw_slew = wrap_pi(yaw_slew + yaw_step);
  yaw_error = wrap_pi(yaw_slew - nav.yaw);
  if (fabs(wrap_pi(cmd.yaw_goal - yaw_slew)) < 0.08 && fabs(yaw_error) < 0.2)
    yaw_i = CLAMP(yaw_i + yaw_error * dt, -YAW_I_LIMIT, YAW_I_LIMIT);
  else
    yaw_i *= 0.98;
  yaw_input = CLAMP(K_YAW_P * yaw_error + K_YAW_I * yaw_i - K_YAW_D * nav.yaw_rate, -YAW_LIMIT, YAW_LIMIT);

  /* Trim only while hovering. During translation the equilibrium tilt is not zero. */
  if (fabs(cmd.v_forward) < 0.15 && fabs(cmd.v_left) < 0.15) {
    roll_i = CLAMP(roll_i + nav.roll * dt, -ATT_I_LIMIT, ATT_I_LIMIT);
    pitch_i = CLAMP(pitch_i + nav.pitch * dt, -ATT_I_LIMIT, ATT_I_LIMIT);
  } else {
    roll_i *= 0.98;
    pitch_i *= 0.98;
  }
  roll_input = K_ROLL_P * CLAMP(nav.roll, -1.0, 1.0) + K_ROLL_I * roll_i + K_ROLL_D * nav.roll_rate +
               roll_disturbance;
  pitch_input = K_PITCH_P * CLAMP(nav.pitch, -1.0, 1.0) + K_PITCH_I * pitch_i + K_PITCH_D * nav.pitch_rate +
                pitch_disturbance;

  front_left = K_VERTICAL_THRUST + vertical_input - roll_input + pitch_input - yaw_input;
  front_right = K_VERTICAL_THRUST + vertical_input + roll_input + pitch_input + yaw_input;
  rear_left = K_VERTICAL_THRUST + vertical_input - roll_input - pitch_input + yaw_input;
  rear_right = K_VERTICAL_THRUST + vertical_input + roll_input - pitch_input - yaw_input;

  front_left = CLAMP(front_left, -MOTOR_LIMIT, MOTOR_LIMIT);
  front_right = CLAMP(front_right, -MOTOR_LIMIT, MOTOR_LIMIT);
  rear_left = CLAMP(rear_left, -MOTOR_LIMIT, MOTOR_LIMIT);
  rear_right = CLAMP(rear_right, -MOTOR_LIMIT, MOTOR_LIMIT);

  wb_motor_set_velocity(motor_fl, front_left);
  wb_motor_set_velocity(motor_fr, -front_right);
  wb_motor_set_velocity(motor_rl, -rear_left);
  wb_motor_set_velocity(motor_rr, rear_right);
}

static void enter_emergency(void) {
  state = EMERGENCY;
}

static void enter_return(double time) {
  state = RETURN_HOME;
  arm_timer(time, home_x, home_y);
}

static void enter_landing(int chase_home) {
  state = LANDING;
  land_chase = chase_home;
  land_stable = 0.0;
}

static void finish_landing(void) {
  motors_enabled = 0;
  motors_off();
  if (!mission_announced) {
    mission_announced = 1;
    if (abort_to_wait) {
      printf("Посадка после прерывания. Можно начать задание заново.\n");
      abort_to_wait = 0;
    } else if (mission_ok && land_chase) {
      printf("Mission completed successfully.\n");
    } else if (!incomplete_reported) {
      fprintf(stderr, "mavic2pro_controller: motors disabled. The mission did not finish the scan.\n");
      incomplete_reported = 1;
    }
    fflush(stdout);
  }
  state = WAIT;
  next_panel_cfg = wb_robot_get_time();
}

static void ensure_panel_defaults(void) {
  if (task_ready)
    return;
  task_kind = TASK_AREA;
  flight_altitude = 1.0;
  task_spacing = 5.0;
  task_width = 8.0;
  task_radius = 6.0;
  task_laps = 1;
  task_cx = 12.0;
  task_cy = 0.0;
  shape_count = 4;
  shape[0].x = -20.0;
  shape[0].y = -20.0;
  shape[1].x = 20.0;
  shape[1].y = -20.0;
  shape[2].x = 20.0;
  shape[2].y = 20.0;
  shape[3].x = -20.0;
  shape[3].y = 20.0;
  task_ready = 1;
}

static void send_panel_cfg(void) {
  char points[900];
  char msg[1200];
  int i;
  int used = 0;
  const char *mode = "area";

  if (task_kind == TASK_ROUTE)
    mode = "route";
  else if (task_kind == TASK_OBJECT)
    mode = "object";
  else if (task_kind == TASK_WATCH)
    mode = "watch";
  points[0] = '\0';
  for (i = 0; i < shape_count; ++i) {
    char one[64];
    int wrote = snprintf(one, sizeof(one), "%s%.2f %.2f", i ? ";" : "", shape[i].x, shape[i].y);
    if (wrote < 0 || used + wrote >= (int)sizeof(points))
      break;
    memcpy(points + used, one, (size_t)wrote + 1);
    used += wrote;
  }
  snprintf(msg, sizeof(msg), "cfg|%s|%.2f|%.2f|%.2f|%.2f|%d|%.2f %.2f|%d|%s", mode, flight_altitude, task_spacing,
           task_width, task_radius, task_laps, task_cx, task_cy, task_repeats, points);
  wb_robot_wwi_send_text("ready patrol");
  wb_robot_wwi_send_text(msg);
}

static int flying_now(void) {
  return state == TAKEOFF || state == HOVER || state == ZIGZAG_SCAN || state == RETURN_HOME || state == LANDING ||
         (state == EMERGENCY && motors_enabled);
}

static void complete_pass(double time) {
  repeat_done++;
  if (repeat_done < task_repeats && waypoint_count > 0) {
    printf("Проход %d из %d выполнен. Начинается следующий.\n", repeat_done, task_repeats);
    fflush(stdout);
    waypoint_index = 0;
    arm_timer(time, waypoints[0].x, waypoints[0].y);
    return;
  }
  mission_ok = 1;
  enter_return(time);
}

static void request_abort(double time) {
  abort_to_wait = 1;
  mission_ok = 0;
  if (!flying_now()) {
    motors_enabled = 0;
    motors_off();
    abort_to_wait = 0;
    state = WAIT;
    printf("Задание прервано. Дрон ждёт на площадке.\n");
    fflush(stdout);
    wb_robot_wwi_send_text("state WAIT");
    return;
  }
  printf("Задание прервано. Возврат и посадка.\n");
  fflush(stdout);
  if (state == LANDING)
    return;
  if (nav.position_reliable)
    enter_return(time);
  else
    enter_landing(0);
}

static int start_from_panel(const char *message) {
  char buffer[1200];
  char *field[10];
  int count = 0;
  char *cursor;
  const char *mode;
  double value;
  int from_air;

  if (strncmp(message, "start|", 6) != 0)
    return 0;
  strncpy(buffer, message, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = '\0';
  cursor = buffer;
  field[count++] = cursor;
  for (; *cursor != '\0' && count < 10; ++cursor) {
    if (*cursor == '|') {
      *cursor = '\0';
      field[count++] = cursor + 1;
    }
  }
  if (count != 10) {
    wb_robot_wwi_send_text("error Неполная команда панели.");
    return 0;
  }
  mode = field[1];
  saw_altitude = saw_spacing = saw_width = saw_radius = saw_laps = saw_center = saw_polygon = saw_path = 0;
  shape_count = 0;
  if (strcmp(mode, "area") == 0)
    task_kind = TASK_AREA;
  else if (strcmp(mode, "route") == 0)
    task_kind = TASK_ROUTE;
  else if (strcmp(mode, "object") == 0)
    task_kind = TASK_OBJECT;
  else if (strcmp(mode, "watch") == 0)
    task_kind = TASK_WATCH;
  else {
    wb_robot_wwi_send_text("error Неизвестный режим. Пульт TX12 выбирается в Запуск.bat.");
    return 0;
  }
  if (!parse_number(field[2], &value) || !(value > 0.0) || value > 80.0) {
    wb_robot_wwi_send_text("error Высота должна быть от 0 до 80 м.");
    return 0;
  }
  flight_altitude = value;
  saw_altitude = 1;
  if (!parse_number(field[3], &task_spacing) || !(task_spacing > 0.0))
    task_spacing = 5.0;
  if (!parse_number(field[4], &task_width) || task_width < 0.0)
    task_width = 0.0;
  if (!parse_number(field[5], &task_radius) || !(task_radius > 0.0))
    task_radius = 6.0;
  if (!parse_number(field[6], &value) || value < 1.0 || value > 20.0)
    task_laps = 1;
  else
    task_laps = (int)(value + 0.5);
  if (!parse_number(field[8], &value) || value < 1.0 || value > 20.0) {
    wb_robot_wwi_send_text("error Повторы задания — число от 1 до 20.");
    return 0;
  }
  task_repeats = (int)(value + 0.5);
  if (field[7][0] == '\0') {
    task_cx = 12.0;
    task_cy = 0.0;
  } else if (!parse_pair(field[7], &task_cx, &task_cy) || !inside_map(task_cx, task_cy)) {
    wb_robot_wwi_send_text("error Центр должен быть парой чисел внутри карты.");
    return 0;
  }
  if (field[9][0] != '\0') {
    char *token = field[9];
    while (token && *token) {
      char *next = strchr(token, ';');
      double x;
      double y;
      if (next)
        *next = '\0';
      trim_line(token);
      if (token[0] && parse_pair(token, &x, &y)) {
        if (!add_shape_point(x, y, 0)) {
          wb_robot_wwi_send_text("error Точка за пределами карты 500 м.");
          return 0;
        }
      }
      token = next ? next + 1 : NULL;
    }
  }
  if (task_kind == TASK_AREA) {
    saw_spacing = 1;
    saw_polygon = 1;
  } else if (task_kind == TASK_ROUTE) {
    saw_spacing = 1;
    saw_width = 1;
    saw_path = 1;
  } else if (task_kind == TASK_OBJECT) {
    saw_radius = 1;
    saw_laps = 1;
    saw_center = 1;
  } else {
    saw_laps = 1;
    saw_polygon = 1;
  }
  if (!task_shape_ok(mode)) {
    wb_robot_wwi_send_text("error Проверьте точки и числа этого режима.");
    return 0;
  }
  flight_z = home_alt + flight_altitude;
  if (!build_task_waypoints()) {
    wb_robot_wwi_send_text("error Маршрут не построен.");
    return 0;
  }
  from_air = flying_now() && isfinite(nav.alt) && nav.alt > home_alt + 0.35;
  waypoint_index = 0;
  repeat_done = 0;
  phase_start = -1.0;
  hover_timer = 0.0;
  mission_ok = 0;
  mission_announced = 0;
  abort_to_wait = 0;
  return_abandoned = 0;
  incomplete_reported = 0;
  fatal_failure = 0;
  if (from_air) {
    motors_enabled = 1;
    arm_timer(wb_robot_get_time(), waypoints[0].x, waypoints[0].y);
    state = ZIGZAG_SCAN;
    printf("Задание начато заново с первой точки.\n");
    fflush(stdout);
    wb_robot_wwi_send_text("state ZIGZAG_SCAN");
  } else {
    state = READY;
    wb_robot_wwi_send_text("state READY");
  }
  return 1;
}

static void poll_panel(void) {
  const char *message;
  while ((message = wb_robot_wwi_receive_text()) != NULL && message[0] != '\0') {
    printf("panel <- %.240s\n", message);
    fflush(stdout);
    if (strcmp(message, "abort") == 0) {
      pending_kind = 2;
    } else if (strncmp(message, "start|", 6) == 0) {
      strncpy(pending_start, message, sizeof(pending_start) - 1);
      pending_start[sizeof(pending_start) - 1] = '\0';
      pending_kind = 1;
    }
  }
}

static void update_mission(double time, double dt) {
  if (state != INITIALIZATION && state != LANDED && state != WAIT) {
    if (bad_attitude_time > ATTITUDE_FAIL_TIME) {
      if (!fatal_reported) {
        fprintf(stderr,
                "mavic2pro_controller: inertial unit or gyro failed. "
                "Motors disabled because attitude can no longer be stabilized.\n");
        fatal_reported = 1;
      }
      fatal_failure = 1;
      motors_enabled = 0;
      enter_emergency();
    } else if (bad_altitude_time > ALTITUDE_FAIL_TIME) {
      if (!fatal_reported) {
        fprintf(stderr,
                "mavic2pro_controller: altitude measurement failed. Motors disabled.\n");
        fatal_reported = 1;
      }
      fatal_failure = 1;
      motors_enabled = 0;
      enter_emergency();
    }
  }

  if (pending_kind == 2) {
    pending_kind = 0;
    request_abort(time);
  } else if (pending_kind == 1) {
    pending_kind = 0;
    if (fatal_failure)
      wb_robot_wwi_send_text("error Датчики отказали, повторный старт невозможен.");
    else if (!start_from_panel(pending_start))
      printf("mavic2pro_controller: the panel command was rejected.\n");
  }

  switch (state) {
    case INITIALIZATION:
      command_hover(nav.yaw, nav.alt);
      if (nav.gps_ok && nav.compass_ok && nav.attitude_ok && nav.accel_ok && isfinite(nav.alt)) {
        home_x = nav.gps_x;
        home_y = nav.gps_y;
        home_alt = nav.alt;
        yaw0 = nav.yaw_compass;
        yaw_slew = yaw0;
        flight_z = home_alt + flight_altitude;
        altitude_cmd = home_alt;
        origin_valid = 1;
        state = WAIT;
        next_panel_cfg = time;
        printf("mavic2pro_controller: waiting for Start in the robot window.\n");
        fflush(stdout);
        send_panel_cfg();
      } else if (time > SENSOR_TIMEOUT) {
        fprintf(stderr,
                "mavic2pro_controller: sensors did not produce a valid sample. Mission stopped.\n");
        fatal_failure = 1;
        motors_enabled = 0;
        enter_emergency();
      }
      break;

    case WAIT:
      motors_enabled = 0;
      command_hover(yaw_slew, home_alt);
      if (time + 1e-9 >= next_panel_cfg) {
        send_panel_cfg();
        next_panel_cfg = time + 2.0;
      }
      break;

    case READY:
      motors_enabled = 1;
      altitude_cmd = home_alt;
      hold_position(home_x, home_y, altitude_cmd, yaw0, HOLD_SPEED, dt);
      state = TAKEOFF;
      break;

    case TAKEOFF:
      if (phase_start < 0.0)
        phase_start = time;
      altitude_ramp(flight_z, CLIMB_RATE, dt);
      hold_position(home_x, home_y, altitude_cmd, yaw0, HOLD_SPEED, dt);
      if (!nav.position_reliable) {
        fprintf(stderr, "mavic2pro_controller: position lost during takeoff.\n");
        mission_ok = 0;
        enter_emergency();
        break;
      }
      if (fabs(altitude_cmd - flight_z) < 0.02 && fabs(nav.alt - flight_z) < ALT_TOL && fabs(nav.vz) < VZ_TOL) {
        hover_timer = 0.0;
        phase_start = time;
        state = HOVER;
      } else if (time - phase_start > 45.0) {
        fprintf(stderr, "mavic2pro_controller: takeoff did not reach the target altitude.\n");
        mission_ok = 0;
        enter_emergency();
      }
      break;

    case HOVER:
      altitude_cmd = flight_z;
      hold_position(home_x, home_y, altitude_cmd, yaw0, HOLD_SPEED, dt);
      if (!nav.position_reliable) {
        fprintf(stderr, "mavic2pro_controller: position lost before the scan.\n");
        mission_ok = 0;
        enter_emergency();
        break;
      }
      if (fabs(nav.alt - flight_z) < ALT_TOL && fabs(nav.vz) < VZ_TOL &&
          hypot(nav.x - home_x, nav.y - home_y) < 0.45 && hypot(nav.vx, nav.vy) < 0.40)
        hover_timer += dt;
      else
        hover_timer = 0.0;
      if (hover_timer >= HOVER_TIME) {
        waypoint_index = 0;
        arm_timer(time, waypoints[0].x, waypoints[0].y);
        state = ZIGZAG_SCAN;
      } else if (time - phase_start > 20.0 && fabs(nav.alt - flight_z) < ALT_TOL) {
        waypoint_index = 0;
        arm_timer(time, waypoints[0].x, waypoints[0].y);
        state = ZIGZAG_SCAN;
      } else if (time - phase_start > 30.0) {
        fprintf(stderr, "mavic2pro_controller: hover did not settle. Scan stopped.\n");
        mission_ok = 0;
        enter_emergency();
      }
      break;

    case ZIGZAG_SCAN:
      altitude_cmd = flight_z;
      if (!nav.position_reliable) {
        mission_ok = 0;
        enter_emergency();
        command_hover(yaw_slew, altitude_cmd);
        break;
      }
      if (waypoint_index >= waypoint_count) {
        complete_pass(time);
        break;
      }
      guide_to(waypoints[waypoint_index].x, waypoints[waypoint_index].y, altitude_cmd, FLIGHT_SPEED, dt);
      if (waypoint_reached(waypoints[waypoint_index].x, waypoints[waypoint_index].y)) {
        waypoint_index++;
        if (waypoint_index >= waypoint_count)
          complete_pass(time);
        else
          arm_timer(time, waypoints[waypoint_index].x, waypoints[waypoint_index].y);
      } else if (time > leg_deadline) {
        fprintf(stderr,
                "mavic2pro_controller: waypoint not reached within the time limit. Scan stopped.\n");
        mission_ok = 0;
        if (nav.position_reliable)
          enter_return(time);
        else
          enter_emergency();
      }
      break;

    case RETURN_HOME:
      altitude_cmd = flight_z;
      if (!nav.position_reliable) {
        fprintf(stderr,
                "mavic2pro_controller: position lost on the way home. "
                "A return to the launch point is not reliable; landing vertically.\n");
        return_abandoned = 1;
        enter_emergency();
        command_hover(yaw_slew, altitude_cmd);
        break;
      }
      guide_to(home_x, home_y, altitude_cmd, FLIGHT_SPEED, dt);
      if (waypoint_reached(home_x, home_y)) {
        enter_landing(1);
      } else if (time > leg_deadline) {
        fprintf(stderr,
                "mavic2pro_controller: home position not reached within the time limit. "
                "Landing vertically.\n");
        return_abandoned = 1;
        mission_ok = 0;
        enter_emergency();
      }
      break;

    case LANDING:
      altitude_ramp(home_alt, DESCENT_RATE, dt);
      if (land_chase && nav.position_reliable)
        hold_position(home_x, home_y, altitude_cmd, yaw_slew, HOLD_SPEED, dt);
      else
        command_hover(yaw_slew, altitude_cmd);
      if (nav.altitude_ok && altitude_cmd <= home_alt + 0.05 && nav.alt <= home_alt + LAND_ALT_BAND &&
          fabs(nav.vz) < 0.20)
        land_stable += dt;
      else
        land_stable = 0.0;
      if (land_stable >= LAND_STABLE_TIME)
        finish_landing();
      break;

    case LANDED:
      command_hover(yaw_slew, home_alt);
      motors_enabled = 0;
      break;

    case EMERGENCY:
      if (fatal_failure || !motors_enabled) {
        motors_enabled = 0;
        command_hover(yaw_slew, altitude_cmd);
        break;
      }
      if (nav.position_reliable && !return_abandoned) {
        enter_return(time);
        hold_position(home_x, home_y, flight_z, yaw_slew, FLIGHT_SPEED, dt);
        break;
      }
      if (nav.altitude_ok) {
        enter_landing(0);
        command_hover(yaw_slew, altitude_cmd);
        break;
      }
      if (!fatal_reported) {
        fprintf(stderr,
                "mavic2pro_controller: safe return is not possible and altitude is unknown. "
                "Motors disabled.\n");
        fatal_reported = 1;
      }
      fatal_failure = 1;
      motors_enabled = 0;
      command_hover(yaw_slew, altitude_cmd);
      break;
  }
}

static void print_telemetry(double time) {
  double x;
  double y;
  double z;

  if (time + 1e-9 < next_telemetry)
    return;

  if (nav.gps_ok) {
    x = nav.gps_x;
    y = nav.gps_y;
    z = nav.gps_z;
  } else {
    x = nav.x;
    y = nav.y;
    z = nav.gps_z;
  }

  printf("Time: %.1f s | State: %s | GPS: X=%.2f Y=%.2f Z=%.2f | Heading: %.1f deg | "
         "Vertical Acceleration: %.2f m/s^2\n",
         time, state_name(state), x, y, z, nav.heading_deg, nav.vert_acc);
  fflush(stdout);
  {
    char panel_state[80];
    int pass_now = repeat_done + 1;
    snprintf(panel_state, sizeof(panel_state), "state %s", state_name(state));
    wb_robot_wwi_send_text(panel_state);
    if (state == WAIT || state == LANDED || state == INITIALIZATION)
      pass_now = 0;
    snprintf(panel_state, sizeof(panel_state), "pass %d %d", pass_now, task_repeats);
    wb_robot_wwi_send_text(panel_state);
  }

  next_telemetry += 1.0;
  if (next_telemetry < time)
    next_telemetry = time + 1.0;
}

int main(void) {
  int time_step;
  double prev_time = 0.0;
  WbDeviceTag motors[4];
  int i;

  wb_robot_init();
  if (!load_task())
    load_zone();
  if (task_failed) {
    fprintf(stderr, "mavic2pro_controller: task file was not used. The panel opens with a default area.\n");
    task_failed = 0;
    task_ready = 0;
  }
  ensure_panel_defaults();
  time_step = (int)wb_robot_get_basic_time_step();
  if (time_step <= 0) {
    fprintf(stderr, "mavic2pro_controller: invalid basicTimeStep. Mission stopped.\n");
    wb_robot_cleanup();
    return EXIT_FAILURE;
  }

  if (!parameters_ok()) {
    wb_robot_cleanup();
    return EXIT_FAILURE;
  }

  gps_dev = wb_robot_get_device("gps");
  compass_dev = wb_robot_get_device("compass");
  accel_dev = wb_robot_get_device("accelerometer");
  gyro_dev = wb_robot_get_device("gyro");
  imu_dev = wb_robot_get_device("inertial unit");
  alt_dev = wb_robot_get_device("altimeter");
  motor_fl = wb_robot_get_device("front left propeller");
  motor_fr = wb_robot_get_device("front right propeller");
  motor_rl = wb_robot_get_device("rear left propeller");
  motor_rr = wb_robot_get_device("rear right propeller");

  if (!device_ok(gps_dev, "gps") || !device_ok(compass_dev, "compass") ||
      !device_ok(accel_dev, "accelerometer") || !device_ok(gyro_dev, "gyro") ||
      !device_ok(imu_dev, "inertial unit") || !device_ok(alt_dev, "altimeter") ||
      !device_ok(motor_fl, "front left propeller") || !device_ok(motor_fr, "front right propeller") ||
      !device_ok(motor_rl, "rear left propeller") || !device_ok(motor_rr, "rear right propeller")) {
    wb_robot_cleanup();
    return EXIT_FAILURE;
  }

  wb_gps_enable(gps_dev, time_step);
  wb_compass_enable(compass_dev, time_step);
  wb_accelerometer_enable(accel_dev, time_step);
  wb_gyro_enable(gyro_dev, time_step);
  wb_inertial_unit_enable(imu_dev, time_step);
  wb_altimeter_enable(alt_dev, time_step);

  motors[0] = motor_fl;
  motors[1] = motor_fr;
  motors[2] = motor_rl;
  motors[3] = motor_rr;
  for (i = 0; i < 4; ++i) {
    wb_motor_set_position(motors[i], INFINITY);
    wb_motor_set_velocity(motors[i], 0.0);
  }

  command_hover(0.0, 0.0);

  while (wb_robot_step(time_step) != -1) {
    const double time = wb_robot_get_time();
    double dt = time - prev_time;
    if (dt <= 0.0 || dt > 0.1)
      dt = time_step / 1000.0;
    prev_time = time;

    update_navigation(dt);
    poll_panel();
    update_mission(time, dt);

    if (motors_enabled && !fatal_failure)
      apply_control(dt);
    else
      motors_off();

    print_telemetry(time);
  }

  motors_off();
  wb_robot_cleanup();
  return origin_valid ? EXIT_SUCCESS : EXIT_FAILURE;
}
