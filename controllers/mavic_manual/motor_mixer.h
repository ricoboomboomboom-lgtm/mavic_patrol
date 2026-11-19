#ifndef MAVIC_MANUAL_MOTOR_MIXER_H
#define MAVIC_MANUAL_MOTOR_MIXER_H

#include "config.h"

/*
 * X mixer for the R2025a Mavic2Pro proto.
 *
 * Body axes (Webots, ENU, yaw 0): +X forward, +Y left, +Z up.
 * Inertial-unit signs used by this mixer:
 *   +roll  = right wing down
 *   +pitch = nose down
 *   +yaw   = nose left (counterclockwise from above)
 *
 * thrustConstants from the proto, and the velocity sign that makes each
 * rotor produce upward thrust:
 *   front left   +0.00026   command applied as-is
 *   front right  -0.00026   command negated
 *   rear left    -0.00026   command negated
 *   rear right   +0.00026   command applied as-is
 *
 * Positive roll_u adds thrust on the right. Positive pitch_u adds thrust
 * on the front (raises a nose-down attitude). Positive yaw_u yaws nose-left.
 * Motor order in the output array: FL, FR, RL, RR, already signed for
 * wb_motor_set_velocity().
 */

void mixer_compute(const FlightConfig *cfg, double collective, double roll_u, double pitch_u, double yaw_u, int enabled,
                   double signed_velocity[4]);
int mixer_self_test(void);

#endif
