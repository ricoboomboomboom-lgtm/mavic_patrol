#include "safety_manager.h"

#include "common.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int flying_state(FlightState state) {
  return state == FS_TAKEOFF || state == FS_FLIGHT || state == FS_POSITION_HOLD || state == FS_LANDING ||
         state == FS_FAILSAFE;
}

static int sticks_moved(const PilotInput *input) {
  return input && input->valid && (input->roll != 0.0 || input->pitch != 0.0);
}

static int usb_stale(const FlightConfig *cfg, const PilotInput *input) {
  if (!input || !input->connected)
    return 1;
  if (!input->valid)
    return input->age_s >= cfg->usb_timeout_s;
  return 0;
}

static const char *arm_block_reason(const FlightConfig *cfg, const PilotInput *input, const SensorFrame *sensors,
                                    const VehicleState *state) {
  if (cfg->diagnostic_mode)
    return "diagnostic mode locks the motors";
  if (cfg->arm_button < 0 && cfg->arm_pov < 0)
    return "arm control is not assigned";
  if (!sensors->motors_present)
    return "one or more propellers are missing";
  if (!input->connected || !input->valid)
    return "joystick is not connected or the axes are not mapped";
  if (!input->map_complete)
    return "roll, pitch, yaw, and throttle axes are not mapped";
  if (!state->att_ok)
    return "attitude is not valid";
  if (!state->rates_ok)
    return "gyro is not valid";
  if (!state->alt_ok || !state->vert_vel_ok)
    return "altitude is not valid";
  if (!state->ground_valid)
    return "ground altitude is still being captured";
  if (fabs(state->roll) > (cfg->arm_max_tilt_deg * 3.14159265358979323846 / 180.0) ||
      fabs(state->pitch) > (cfg->arm_max_tilt_deg * 3.14159265358979323846 / 180.0))
    return "vehicle is not level";
  if (!input->safe_sticks)
    return "sticks are not at neutral, or throttle is outside its calibrated deadzone";
  return NULL;
}

static void enter_state(SafetyManager *safety, FlightState next, const char *detail) {
  char buffer[160];

  buffer[0] = '\0';
  if (detail)
    snprintf(buffer, sizeof(buffer), "%s", detail);
  safety->state = next;
  safety->state_time = 0.0;
  safety->gate_time = 0.0;
  safety->hold_ready_time = 0.0;
  safety->contact_time = 0.0;
  safety->heuristic_time = 0.0;
  safety->handover_time = 0.0;
  snprintf(safety->detail, sizeof(safety->detail), "%s", buffer);
  printf("mavic_manual: state -> %s (%s)\n", flight_state_name(next), safety->detail);
}

static int confirm_touchdown(SafetyManager *safety, const FlightConfig *cfg, const SensorFrame *sensors,
                             const VehicleState *state, int descent_commanded, double dt) {
  int slow = state->vert_vel_ok && fabs(state->vz) <= cfg->contact_vz_max;
  int low = state->alt_ok && state->ground_valid && (state->z <= (state->ground_z + cfg->landing_enter_height));

  if (!safety->left_ground) {
    safety->contact_time = 0.0;
    safety->heuristic_time = 0.0;
    return 0;
  }

  if (sensors->supervisor && sensors->ground_contact && slow && (low || !state->alt_ok))
    safety->contact_time += dt;
  else
    safety->contact_time = 0.0;

  if (safety->contact_time >= cfg->ground_contact_time)
    return 1;

  /*
   * Without supervisor contact points this is only a supporting hint.
   * It still requires a commanded descent, low altitude, and near-zero
   * vertical speed. Reaching an altitude by itself is not accepted.
   */
  if (!sensors->supervisor && descent_commanded && low && slow && state->pos_ok && hypot(state->vx, state->vy) < 0.4)
    safety->heuristic_time += dt;
  else
    safety->heuristic_time = 0.0;
  return safety->heuristic_time >= 0.80;
}

static void update_lateral(SafetyManager *safety, const FlightConfig *cfg, const PilotInput *input, const VehicleState *state,
                           double dt) {
  double speed = hypot(state->vx, state->vy);

  if (!state->pos_ok || !state->att_ok) {
    if (sticks_moved(input) && state->att_ok)
      safety->lateral = LAT_MANUAL;
    else
      safety->lateral = LAT_LEVEL;
    safety->hold_ready_time = 0.0;
    return;
  }

  if (sticks_moved(input)) {
    safety->lateral = LAT_MANUAL;
    safety->hold_ready_time = 0.0;
    return;
  }

  if (safety->lateral == LAT_MANUAL || safety->lateral == LAT_BRAKE) {
    safety->lateral = LAT_BRAKE;
    if (speed < cfg->brake_speed_enter_hold)
      safety->hold_ready_time += dt;
    else
      safety->hold_ready_time = 0.0;
    if (safety->hold_ready_time >= cfg->brake_hold_time) {
      safety->lateral = LAT_HOLD;
      safety->latch_position_pulse = 1;
      safety->hold_ready_time = 0.0;
    }
    return;
  }

  safety->lateral = LAT_HOLD;
}

void safety_init(SafetyManager *safety) {
  if (!safety)
    return;
  memset(safety, 0, sizeof(*safety));
  safety->state = FS_INITIALIZATION;
  safety->lateral = LAT_LEVEL;
  snprintf(safety->detail, sizeof(safety->detail), "waiting for devices");
}

void safety_update(SafetyManager *safety, const FlightConfig *cfg, const PilotInput *input, const SensorFrame *sensors,
                   const VehicleState *state, double dt, SafetyStatus *status) {
  FlightState next;
  const char *block;
  int lost;
  int descent_commanded;
  int low;
  VertMode vertical;
  int motors_live;
  double authority;

  if (!safety || !cfg || !input || !sensors || !state || !status)
    return;
  if (is_bad(dt) || dt <= 0.0)
    dt = 0.008;

  safety->latch_position_pulse = 0;
  safety->latch_altitude_pulse = 0;
  next = safety->state;
  lost = usb_stale(cfg, input);
  low = state->alt_ok && state->ground_valid && (state->z <= (state->ground_z + cfg->landing_enter_height));
  descent_commanded = 0;
  block = arm_block_reason(cfg, input, sensors, state);

  if (safety->block_arm_until_release && !input->arm_held)
    safety->block_arm_until_release = 0;

  if (state->alt_ok && state->ground_valid && state->z > (state->ground_z + cfg->liftoff_clearance))
    safety->left_ground = 1;

  if (flying_state(safety->state) && !state->att_ok) {
    safety->fail_reason |= FAIL_ATTITUDE;
    if (safety->state != FS_FAILSAFE)
      next = FS_FAILSAFE;
    snprintf(safety->detail, sizeof(safety->detail), "attitude lost; open-loop collective, landing not assured");
  }

  switch (safety->state) {
    case FS_INITIALIZATION:
      if (!sensors->motors_present) {
        snprintf(safety->detail, sizeof(safety->detail), "propellers missing; motors stay off");
      } else if (safety->state_time > 0.20) {
        next = cfg->diagnostic_mode ? FS_DIAGNOSTIC : FS_READY;
        snprintf(safety->detail, sizeof(safety->detail),
                 cfg->diagnostic_mode ? "diagnostic lock; motors stay off" : "sensors are producing finite data");
      }
      break;

    case FS_DIAGNOSTIC:
      snprintf(safety->detail, sizeof(safety->detail), "motors locked; set diagnostic_mode = 0 after mapping the sticks");
      break;

    case FS_READY:
      if (state->att_ok && state->rates_ok && state->alt_ok && sensors->motors_present)
        safety->gate_time += dt;
      else
        safety->gate_time = 0.0;
      if (safety->gate_time > 0.20)
        next = FS_DISARMED;
      else
        snprintf(safety->detail, sizeof(safety->detail), "waiting for a stable attitude and altitude");
      break;

    case FS_DISARMED:
      if (!safety->block_arm_until_release && input->arm_rise && block == NULL)
        next = FS_ARMING;
      else if (block)
        snprintf(safety->detail, sizeof(safety->detail), "%s", block);
      else
        snprintf(safety->detail, sizeof(safety->detail), "motors off, waiting for a new arm edge");
      break;

    case FS_ARMING:
      if (lost || block != NULL) {
        next = FS_DISARMED;
        snprintf(safety->detail, sizeof(safety->detail), "%s", block ? block : "joystick lost during arming");
      } else if (safety->state_time >= cfg->arming_time) {
        next = FS_ARMED;
      }
      break;

    case FS_ARMED:
      if (lost || input->disarm_rise || !state->att_ok) {
        next = FS_DISARMED;
        snprintf(safety->detail, sizeof(safety->detail), "disarmed before takeoff");
      } else if (safety->state_time >= cfg->armed_timeout) {
        next = FS_DISARMED;
        snprintf(safety->detail, sizeof(safety->detail), "arm timeout; motors shut down");
      } else if (input->takeoff_rise && block == NULL && (cfg->takeoff_button >= 0 || cfg->takeoff_pov >= 0)) {
        next = FS_TAKEOFF;
      } else if (cfg->takeoff_button < 0 && cfg->takeoff_pov < 0) {
        snprintf(safety->detail, sizeof(safety->detail), "armed idle; takeoff control is not assigned");
      } else {
        snprintf(safety->detail, sizeof(safety->detail), "armed idle; waiting for takeoff");
      }
      break;

    case FS_TAKEOFF:
      safety->lateral = (state->pos_ok && state->att_ok) ? LAT_HOLD : LAT_LEVEL;
      if (lost) {
        safety->fail_reason |= FAIL_USB;
        next = FS_FAILSAFE;
        snprintf(safety->detail, sizeof(safety->detail), "USB lost during takeoff");
      } else if (state->alt_ok && state->z >= (safety->takeoff_target_z - 0.08) && fabs(state->vz) < 0.15) {
        safety->gate_time += dt;
        if (safety->gate_time >= 0.30)
          next = FS_POSITION_HOLD;
      } else {
        safety->gate_time = 0.0;
      }
      break;

    case FS_FLIGHT:
    case FS_POSITION_HOLD:
      if (lost) {
        safety->fail_reason |= FAIL_USB;
        next = FS_FAILSAFE;
        snprintf(safety->detail, sizeof(safety->detail), "USB lost; stale sticks discarded");
      } else {
        update_lateral(safety, cfg, input, state, dt);
        if (input->valid && input->throttle < 0.0 && low)
          next = FS_LANDING;
        else
          next = (safety->lateral == LAT_HOLD) ? FS_POSITION_HOLD : FS_FLIGHT;
      }
      break;

    case FS_LANDING:
      safety->lateral = (state->pos_ok && state->att_ok) ? LAT_HOLD : LAT_LEVEL;
      descent_commanded = input->valid && input->throttle < 0.0;
      if (lost) {
        safety->fail_reason |= FAIL_USB;
        next = FS_FAILSAFE;
        snprintf(safety->detail, sizeof(safety->detail), "USB lost during landing");
      } else if (input->valid && input->throttle > 0.0) {
        next = FS_FLIGHT;
        safety->lateral = LAT_MANUAL;
      } else if (input->valid && input->throttle == 0.0 && !low) {
        next = FS_POSITION_HOLD;
        safety->lateral = state->pos_ok ? LAT_HOLD : LAT_LEVEL;
        safety->latch_altitude_pulse = 1;
        safety->latch_position_pulse = 1;
      }
      break;

    case FS_FAILSAFE:
      safety->lateral = (state->pos_ok && state->att_ok && !(safety->fail_reason & FAIL_ATTITUDE)) ? LAT_HOLD : LAT_LEVEL;
      descent_commanded = 1;
      if ((safety->fail_reason & FAIL_USB) && !lost && input->valid && input->safe_sticks && state->att_ok &&
          !(safety->fail_reason & FAIL_ATTITUDE)) {
        safety->handover_time += dt;
        snprintf(safety->detail, sizeof(safety->detail), "USB back; holding for a neutral-stick handover");
      } else {
        safety->handover_time = 0.0;
        if ((safety->fail_reason & FAIL_USB) && input->valid && !input->safe_sticks)
          snprintf(safety->detail, sizeof(safety->detail), "USB back, but sticks are not neutral; still landing");
        else if (safety->fail_reason & FAIL_ATTITUDE)
          snprintf(safety->detail, sizeof(safety->detail), "attitude failed; descent is not fully stabilized");
        else
          snprintf(safety->detail, sizeof(safety->detail), "failsafe descent");
      }
      if (safety->handover_time >= cfg->handover_valid_s) {
        safety->fail_reason &= ~FAIL_USB;
        safety->lateral = state->pos_ok ? LAT_HOLD : LAT_LEVEL;
        safety->latch_position_pulse = 1;
        safety->latch_altitude_pulse = 1;
        next = FS_POSITION_HOLD;
        snprintf(safety->detail, sizeof(safety->detail), "control returned to the pilot");
      }
      break;

    case FS_LANDED:
      if (!safety->block_arm_until_release && input->arm_rise && block == NULL)
        next = FS_ARMING;
      else
        snprintf(safety->detail, sizeof(safety->detail), "on the ground; motors stay off until a new arm command");
      break;

    default:
      next = FS_INITIALIZATION;
      break;
  }

  if (flying_state(safety->state) || next == FS_FAILSAFE || safety->state == FS_LANDING) {
    int commanded = descent_commanded || safety->state == FS_FAILSAFE || next == FS_FAILSAFE;
    if (confirm_touchdown(safety, cfg, sensors, state, commanded, dt)) {
      next = FS_LANDED;
      safety->fail_reason = FAIL_NONE;
      snprintf(safety->detail, sizeof(safety->detail), "ground contact confirmed");
    }
  }

  if (next != safety->state) {
    if (next == FS_TAKEOFF) {
      safety->takeoff_target_z = state->ground_z + cfg->takeoff_height;
      safety->lateral = LAT_HOLD;
      safety->latch_position_pulse = 1;
      safety->latch_altitude_pulse = 1;
      safety->left_ground = 0;
      snprintf(safety->detail, sizeof(safety->detail), "takeoff toward z=%.2f", safety->takeoff_target_z);
    }
    if (next == FS_POSITION_HOLD && safety->state == FS_TAKEOFF) {
      safety->lateral = state->pos_ok ? LAT_HOLD : LAT_LEVEL;
      safety->latch_position_pulse = 1;
      safety->latch_altitude_pulse = 1;
    }
    if (next == FS_FAILSAFE) {
      safety->lateral = (state->pos_ok && state->att_ok) ? LAT_HOLD : LAT_LEVEL;
      safety->latch_position_pulse = state->pos_ok ? 1 : 0;
      safety->latch_altitude_pulse = 0;
    }
    if (next == FS_LANDED || next == FS_DISARMED) {
      safety->block_arm_until_release = 1;
      safety->left_ground = 0;
      safety->lateral = LAT_LEVEL;
    }
    if (next == FS_ARMING)
      snprintf(safety->detail, sizeof(safety->detail), "arming");
    if (next == FS_ARMED && safety->detail[0] == '\0')
      snprintf(safety->detail, sizeof(safety->detail), "motors idle");
    enter_state(safety, next, safety->detail);
  } else {
    safety->state_time += dt;
  }

  if (safety->state == FS_TAKEOFF || safety->state == FS_ARMING || safety->state == FS_ARMED)
    vertical = (safety->state == FS_TAKEOFF) ? VERT_TAKEOFF : VERT_IDLE;
  else if (safety->state == FS_FAILSAFE) {
    if (!state->att_ok)
      vertical = state->alt_ok ? VERT_DEGRADED : VERT_DEGRADED;
    else if (!state->alt_ok || !state->vert_vel_ok) {
      safety->fail_reason |= FAIL_ALTITUDE;
      vertical = VERT_DEGRADED;
    } else {
      vertical = VERT_FAILSAFE;
    }
  } else if (safety->state == FS_LANDING) {
    vertical = (state->alt_ok && state->vert_vel_ok) ? VERT_LAND : VERT_DEGRADED;
  } else if (safety->state == FS_FLIGHT || safety->state == FS_POSITION_HOLD) {
    if (!state->alt_ok || !state->vert_vel_ok)
      vertical = VERT_DEGRADED;
    else if (input->valid && input->throttle != 0.0)
      vertical = VERT_SPEED;
    else
      vertical = VERT_HOLD;
  } else {
    vertical = VERT_OFF;
  }

  if (safety->state == FS_ARMING || safety->state == FS_ARMED || flying_state(safety->state)) {
    safety->spun = 1;
    safety->shutdown_time = 0.0;
    motors_live = 1;
  } else if (safety->spun && safety->shutdown_time < 2.0) {
    safety->shutdown_time += dt;
    motors_live = 1;
    vertical = VERT_OFF;
  } else {
    motors_live = 0;
    vertical = VERT_OFF;
  }

  if (safety->state == FS_ARMED || safety->state == FS_ARMING)
    authority = state->att_ok ? 0.45 : 0.0;
  else if (flying_state(safety->state) && state->att_ok)
    authority = (safety->fail_reason & FAIL_ATTITUDE) ? 0.0 : 1.0;
  else
    authority = 0.0;

  memset(status, 0, sizeof(*status));
  status->state = safety->state;
  status->lateral = safety->lateral;
  status->vertical = vertical;
  status->fail_reason = safety->fail_reason;
  status->motors_live = motors_live;
  status->left_ground = safety->left_ground;
  status->latch_position = safety->latch_position_pulse;
  status->latch_altitude = safety->latch_altitude_pulse;
  status->pilot_sticks_enabled =
      (safety->state == FS_FLIGHT || safety->state == FS_POSITION_HOLD || safety->state == FS_LANDING) ? 1 : 0;
  status->attitude_authority = authority;
  status->takeoff_target_z = safety->takeoff_target_z;
  status->arm_block = block;
  snprintf(status->detail, sizeof(status->detail), "%s", safety->detail);

  if (!state->pos_ok && flying_state(safety->state) && safety->lateral == LAT_LEVEL &&
      !(safety->fail_reason & FAIL_USB)) {
    snprintf(status->detail, sizeof(status->detail), "GPS unavailable; position hold is off, attitude control remains");
  }
}
