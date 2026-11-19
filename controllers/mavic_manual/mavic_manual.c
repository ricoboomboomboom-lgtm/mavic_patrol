/*
 * Manual position-hold controller for the Webots R2025a DJI Mavic 2 Pro.
 *
 * This is an independent cascade controller. It is not DJI firmware and it
 * does not reproduce a proprietary flight stack. Motor signs follow the
 * public Mavic2Pro.proto thrust constants and the stock Webots sample mixer.
 *
 * Loop, after every wb_robot_step():
 *   sensors -> state estimate -> joystick -> safety state machine
 *   -> position / altitude / attitude -> mixer -> motors -> telemetry
 */

#include "altitude_controller.h"
#include "attitude_controller.h"
#include "common.h"
#include "config.h"
#include "flight_types.h"
#include "input_manager.h"
#include "motor_mixer.h"
#include "pid.h"
#include "position_controller.h"
#include "safety_manager.h"
#include "sensor_manager.h"
#include "state_estimator.h"
#include "telemetry.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <webots/robot.h>

static int run_self_tests(void) {
  int failed = 0;
  if (common_self_test()) {
    printf("mavic_manual: SELF-CHECK common FAILED\n");
    failed = 1;
  }
  if (pid_self_test()) {
    printf("mavic_manual: SELF-CHECK pid FAILED\n");
    failed = 1;
  }
  if (mixer_self_test()) {
    printf("mavic_manual: SELF-CHECK mixer FAILED\n");
    failed = 1;
  }
  if (position_self_test()) {
    printf("mavic_manual: SELF-CHECK speed limit FAILED\n");
    failed = 1;
  }
  if (!failed)
    printf("mavic_manual: numeric self-checks passed (not a flight test)\n");
  return failed;
}

int main(int argc, char **argv) {
  int timestep;
  double step_s;
  char config_path[512];
  char config_error[160];
  FlightConfig cfg;
  InputManager input;
  PilotInput pilot;
  SensorFrame sensors;
  StateEstimator estimator;
  VehicleState vehicle;
  AttitudeController attitude;
  AltitudeController altitude;
  PositionController position;
  SafetyManager safety;
  SafetyStatus status;
  Telemetry telemetry;
  ControlTrace trace;
  double previous_time = -1.0;

  (void)argc;
  (void)argv;

  wb_robot_init();
  timestep = (int)wb_robot_get_basic_time_step();
  if (timestep <= 0)
    timestep = 8;
  step_s = ((double)timestep) / 1000.0;

  config_resolve_path(config_path, (int)sizeof(config_path));
  config_load(&cfg, config_path, config_error, (int)sizeof(config_error));
  if (config_error[0])
    printf("mavic_manual: %s\n", config_error);
  if (fabs((double)timestep - cfg.expected_timestep_ms) > 0.1)
    printf("mavic_manual: world timestep is %d ms; config expected %.1f ms. The loop follows the world.\n", timestep,
           cfg.expected_timestep_ms);

  if (run_self_tests()) {
    printf("mavic_manual: refusing to run because a numeric self-check failed\n");
    wb_robot_cleanup();
    return EXIT_FAILURE;
  }

  sensors_init(timestep, &cfg);
  input_init(&input, timestep);
  estimator_init(&estimator);
  attitude_init(&attitude);
  altitude_init(&altitude, &cfg);
  position_init(&position);
  safety_init(&safety);
  telemetry_init(&telemetry);
  memset(&trace, 0, sizeof(trace));
  memset(&status, 0, sizeof(status));

  printf("mavic_manual: position-hold controller ready. diagnostic_mode=%d\n", cfg.diagnostic_mode);
  printf("mavic_manual: arm_button=%d takeoff_button=%d disarm_button=%d\n", cfg.arm_button, cfg.takeoff_button,
         cfg.disarm_button);
  printf("mavic_manual: axes roll=%d pitch=%d yaw=%d throttle=%d ( -1 means unmapped )\n", cfg.roll_axis, cfg.pitch_axis,
         cfg.yaw_axis, cfg.throttle_axis);

  while (wb_robot_step(timestep) != -1) {
    double now = wb_robot_get_time();
    double dt = step_s;
    double stick_roll = 0.0;
    double stick_pitch = 0.0;
    double stick_yaw = 0.0;
    double stick_throttle = 0.0;
    int airborne;

    if (previous_time >= 0.0) {
      dt = now - previous_time;
      if (is_bad(dt) || dt <= 0.0 || dt > 0.05)
        dt = step_s;
    }
    previous_time = now;

    sensors_read(&sensors, &cfg);
    airborne = (safety.state == FS_TAKEOFF || safety.state == FS_FLIGHT || safety.state == FS_POSITION_HOLD ||
                safety.state == FS_LANDING || safety.state == FS_FAILSAFE);
    estimator_update(&estimator, &cfg, &sensors, &vehicle, airborne, dt);
    input_update(&input, &cfg, &pilot, dt);
    safety_update(&safety, &cfg, &pilot, &sensors, &vehicle, dt, &status);

    if (status.pilot_sticks_enabled) {
      stick_roll = pilot.roll;
      stick_pitch = pilot.pitch;
      stick_yaw = pilot.yaw;
      stick_throttle = pilot.throttle;
    }

    memset(&trace, 0, sizeof(trace));
    position_update(&position, &cfg, &vehicle, status.lateral, stick_roll, stick_pitch, status.latch_position, dt, &trace);
    altitude_update(&altitude, &cfg, &vehicle, status.vertical, stick_throttle, status.latch_altitude, status.takeoff_target_z,
                    dt, &trace);
    attitude_update(&attitude, &cfg, &vehicle, trace.roll_sp, trace.pitch_sp, stick_yaw, status.pilot_sticks_enabled,
                    (status.attitude_authority > 0.5) ? 1 : (status.attitude_authority > 0.0 ? 1 : 0), dt, &trace);
    if (status.attitude_authority > 0.0 && status.attitude_authority < 0.99) {
      trace.roll_u *= status.attitude_authority;
      trace.pitch_u *= status.attitude_authority;
      trace.yaw_u *= status.attitude_authority;
    }
    mixer_compute(&cfg, trace.collective, trace.roll_u, trace.pitch_u, trace.yaw_u, status.motors_live, trace.motor);
    if (!status.motors_live) {
      trace.motor[0] = 0.0;
      trace.motor[1] = 0.0;
      trace.motor[2] = 0.0;
      trace.motor[3] = 0.0;
    }
    motors_write(trace.motor);
    telemetry_update(&telemetry, &cfg, &input, &pilot, &sensors, &vehicle, &status, &trace, now);
  }

  {
    double stopped[4] = {0.0, 0.0, 0.0, 0.0};
    motors_write(stopped);
  }
  wb_robot_cleanup();
  return EXIT_SUCCESS;
}
