#include "telemetry.h"

#include "common.h"
#include "sensor_manager.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static double deg(double rad) {
  return rad * 180.0 / M_PI;
}

void telemetry_init(Telemetry *telemetry) {
  if (!telemetry)
    return;
  memset(telemetry, 0, sizeof(*telemetry));
  telemetry->last_state = FS_INITIALIZATION;
  telemetry->last_block[0] = '\0';
}

static void print_buttons(const PilotInput *pilot) {
  int i;
  int any = 0;
  printf("  buttons:");
  for (i = 0; i < BUTTON_CAP; ++i) {
    if (pilot->button[i]) {
      printf(" %d", i);
      any = 1;
    }
  }
  if (!any)
    printf(" none");
  if (pilot->button_overflow)
    printf(" (id>=%d also down)", BUTTON_CAP);
  printf("\n");
}

static void print_suggestion(const FlightConfig *cfg, const InputManager *input, const PilotInput *pilot, double time) {
  int i;
  int any = 0;
  static int hints_printed = 0;

  for (i = 0; i < pilot->axis_count && i < AXIS_CAP; ++i) {
    if (input->moved[i])
      any = 1;
  }
  if (any) {
    printf("  paste candidates (release the stick at the desired neutral before copying):\n");
    for (i = 0; i < pilot->axis_count && i < AXIS_CAP; ++i) {
      if (!input->moved[i])
        continue;
      printf("    axis %d neutral=%.0f min=%.0f max=%.0f\n", i, pilot->raw[i], input->min_seen[i], input->max_seen[i]);
    }
  }
  if (!hints_printed && time > 0.4) {
    hints_printed = 1;
    if (cfg->arm_button < 0 && cfg->arm_pov < 0)
      printf("  arm_button is unset. Press the switch you want and use the button id printed above.\n");
    if (cfg->takeoff_button < 0 && cfg->takeoff_pov < 0)
      printf("  takeoff_button is unset. It must be a different press from arm.\n");
  }
}

static void leds_for_state(FlightState state, double time) {
  int blink_slow = ((int)(time * 2.0)) % 2;
  int blink_fast = ((int)(time * 8.0)) % 2;
  switch (state) {
    case FS_DIAGNOSTIC:
      leds_set(blink_slow, blink_slow);
      break;
    case FS_ARMING:
    case FS_ARMED:
      leds_set(1, 1);
      break;
    case FS_TAKEOFF:
    case FS_FLIGHT:
    case FS_POSITION_HOLD:
      leds_set(blink_slow, !blink_slow);
      break;
    case FS_LANDING:
      leds_set(blink_slow, blink_slow);
      break;
    case FS_FAILSAFE:
      leds_set(blink_fast, blink_fast);
      break;
    default:
      leds_set(0, 0);
      break;
  }
}

void telemetry_update(Telemetry *telemetry, const FlightConfig *cfg, const InputManager *input, const PilotInput *pilot,
                      const SensorFrame *sensors, const VehicleState *state, const SafetyStatus *safety,
                      const ControlTrace *trace, double time) {
  double period;
  int diagnostic;
  const char *block;

  if (!telemetry || !cfg || !input || !pilot || !sensors || !state || !safety || !trace)
    return;

  leds_for_state(safety->state, time);
  diagnostic = (safety->state == FS_DIAGNOSTIC) || cfg->diagnostic_mode;
  period = diagnostic ? cfg->diagnostic_period : cfg->telemetry_period;

  block = safety->arm_block ? safety->arm_block : "";
  if (safety->state != telemetry->last_state) {
    telemetry->last_state = safety->state;
    telemetry->next_print = time;
  }
  if (strcmp(block, telemetry->last_block) != 0 && block[0] != '\0' &&
      (safety->state == FS_DISARMED || safety->state == FS_READY || safety->state == FS_LANDED)) {
    printf("mavic_manual: arm blocked: %s\n", block);
    snprintf(telemetry->last_block, sizeof(telemetry->last_block), "%s", block);
  }

  if (time + 1e-9 < telemetry->next_print)
    return;
  telemetry->next_print = time + period;

  if (diagnostic && (safety->state == FS_DIAGNOSTIC || safety->state == FS_INITIALIZATION || safety->state == FS_READY ||
                     safety->state == FS_DISARMED)) {
    int i;
    printf("[DIAG] t=%.2f connected=%d model=\"%s\" axes=%d povs=%d map=%d valid=%d age=%.2f\n", time, pilot->connected,
           pilot->model, pilot->axis_count, pilot->pov_count, pilot->map_complete, pilot->valid, pilot->age_s);
    printf("  semantic roll=%+.2f pitch=%+.2f yaw=%+.2f throttle=%+.2f safe=%d\n", pilot->roll, pilot->pitch, pilot->yaw,
           pilot->throttle, pilot->safe_sticks);
    for (i = 0; i < pilot->axis_count && i < AXIS_CAP; ++i) {
      const char *dir = "still";
      if (input->direction[i] > 0)
        dir = "rising";
      else if (input->direction[i] < 0)
        dir = "falling";
      printf("  ax%02d raw=%8.0f min=%8.0f max=%8.0f noise=%6.1f %s\n", i, pilot->raw[i], input->min_seen[i],
             input->max_seen[i], input->abs_delta_avg[i], dir);
    }
    print_buttons(pilot);
    for (i = 0; i < pilot->pov_count && i < POV_CAP; ++i)
      printf("  pov%d=%d\n", i, pilot->pov[i]);
    print_suggestion(cfg, input, pilot, time);
    printf("  motor cmd %+.1f %+.1f %+.1f %+.1f\n", trace->motor[0], trace->motor[1], trace->motor[2], trace->motor[3]);
    printf("  sensors gps=%d imu=%d gyro=%d compass=%d accel=%d alt=%d contact=%d ground_contact=%d\n", sensors->gps_finite,
           sensors->imu_finite, sensors->gyro_finite, sensors->compass_finite, sensors->accel_finite,
           sensors->altimeter_finite, sensors->contact_count, sensors->ground_contact);
    printf("  est rpy_deg=%+.1f %+.1f %+.1f z=%.2f vz=%.2f ground=%d faults=0x%x\n", deg(state->roll), deg(state->pitch),
           deg(state->yaw), state->z, state->vz, state->ground_valid, state->fault_bits);
    if (sensors->accel_finite)
      printf("  accel xyz=%.2f %.2f %.2f (monitored, not used in the control law)\n", sensors->accel_x, sensors->accel_y,
             sensors->accel_z);
    return;
  }

  printf("[FC] t=%.2f %s lat=%s vert=%s fail=0x%x motors=%d\n", time, flight_state_name(safety->state),
         lateral_mode_name(safety->lateral), vertical_mode_name(safety->vertical), safety->fail_reason, safety->motors_live);
  printf("  %s\n", safety->detail);
  printf("  rpy_deg=%+.1f %+.1f %+.1f rates=%+.2f %+.2f %+.2f\n", deg(state->roll), deg(state->pitch), deg(state->yaw),
         state->roll_rate, state->pitch_rate, state->yaw_rate);
  printf("  pos=%.2f %.2f z=%.2f vel=%.2f %.2f %.2f body_f/l=%.2f %.2f\n", state->x, state->y, state->z, state->vx, state->vy,
         state->vz, state->v_forward, state->v_left);
  printf("  sp_rp_deg=%+.1f %+.1f yaw_deg=%+.1f yaw_rate=%+.2f z=%.2f vz=%.2f\n", deg(trace->roll_sp), deg(trace->pitch_sp),
         deg(trace->yaw_sp), trace->yaw_rate_sp, trace->z_sp, trace->vz_sp);
  printf("  mix T=%.1f r=%.2f p=%.2f y=%.2f motors=%+.1f %+.1f %+.1f %+.1f\n", trace->collective, trace->roll_u,
         trace->pitch_u, trace->yaw_u, trace->motor[0], trace->motor[1], trace->motor[2], trace->motor[3]);
  printf("  joy valid=%d age=%.2f sticks=%+.2f %+.2f %+.2f %+.2f sensors att=%d rate=%d pos=%d alt=%d vz=%d\n", pilot->valid,
         pilot->age_s, pilot->roll, pilot->pitch, pilot->yaw, pilot->throttle, state->att_ok, state->rates_ok, state->pos_ok,
         state->alt_ok, state->vert_vel_ok);
}
