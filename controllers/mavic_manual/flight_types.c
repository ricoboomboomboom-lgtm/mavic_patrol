#include "flight_types.h"

const char *flight_state_name(FlightState state) {
  switch (state) {
    case FS_INITIALIZATION:
      return "INITIALIZATION";
    case FS_DIAGNOSTIC:
      return "DIAGNOSTIC";
    case FS_READY:
      return "READY";
    case FS_DISARMED:
      return "DISARMED";
    case FS_ARMING:
      return "ARMING";
    case FS_ARMED:
      return "ARMED";
    case FS_TAKEOFF:
      return "TAKEOFF";
    case FS_FLIGHT:
      return "FLIGHT";
    case FS_POSITION_HOLD:
      return "POSITION_HOLD";
    case FS_LANDING:
      return "LANDING";
    case FS_FAILSAFE:
      return "FAILSAFE";
    case FS_LANDED:
      return "LANDED";
    default:
      return "UNKNOWN";
  }
}

const char *lateral_mode_name(LateralMode mode) {
  switch (mode) {
    case LAT_LEVEL:
      return "LEVEL";
    case LAT_MANUAL:
      return "MANUAL";
    case LAT_BRAKE:
      return "BRAKE";
    case LAT_HOLD:
      return "HOLD";
    default:
      return "UNKNOWN";
  }
}

const char *vertical_mode_name(VertMode mode) {
  switch (mode) {
    case VERT_OFF:
      return "OFF";
    case VERT_IDLE:
      return "IDLE";
    case VERT_TAKEOFF:
      return "TAKEOFF";
    case VERT_HOLD:
      return "HOLD";
    case VERT_SPEED:
      return "SPEED";
    case VERT_LAND:
      return "LAND";
    case VERT_FAILSAFE:
      return "FAILSAFE";
    case VERT_DEGRADED:
      return "DEGRADED";
    default:
      return "UNKNOWN";
  }
}
