#include "motor_mixer.h"

#include "common.h"

#include <math.h>
#include <string.h>

void mixer_compute(const FlightConfig *cfg, double collective, double roll_u, double pitch_u, double yaw_u, int enabled,
                   double signed_velocity[4]) {
  double limit;
  double fl;
  double fr;
  double rl;
  double rr;

  if (!signed_velocity)
    return;
  signed_velocity[0] = 0.0;
  signed_velocity[1] = 0.0;
  signed_velocity[2] = 0.0;
  signed_velocity[3] = 0.0;
  if (!enabled || !cfg)
    return;

  limit = fabs(cfg->motor_command_limit);
  if (limit < 1.0)
    limit = 1.0;

  collective = clamp_d(finite_or(collective, 0.0), 0.0, limit);
  roll_u = finite_or(roll_u, 0.0);
  pitch_u = finite_or(pitch_u, 0.0);
  yaw_u = finite_or(yaw_u, 0.0);

  fl = collective - roll_u + pitch_u - yaw_u;
  fr = collective + roll_u + pitch_u + yaw_u;
  rl = collective - roll_u - pitch_u + yaw_u;
  rr = collective + roll_u - pitch_u - yaw_u;

  /* Keep the propellers from reversing. Reversed thrust flips the yaw torque. */
  fl = clamp_d(fl, 0.0, limit);
  fr = clamp_d(fr, 0.0, limit);
  rl = clamp_d(rl, 0.0, limit);
  rr = clamp_d(rr, 0.0, limit);

  signed_velocity[0] = fl;
  signed_velocity[1] = -fr;
  signed_velocity[2] = -rl;
  signed_velocity[3] = rr;
}

int mixer_self_test(void) {
  FlightConfig cfg;
  double motor[4];

  memset(&cfg, 0, sizeof(cfg));
  cfg.motor_command_limit = 200.0;
  mixer_compute(&cfg, 68.5, 0.0, 0.0, 0.0, 1, motor);
  if (fabs(motor[0] - 68.5) > 1e-6)
    return 1;
  if (fabs(motor[1] + 68.5) > 1e-6)
    return 1;
  if (fabs(motor[2] + 68.5) > 1e-6)
    return 1;
  if (fabs(motor[3] - 68.5) > 1e-6)
    return 1;

  mixer_compute(&cfg, 68.5, 10.0, 0.0, 0.0, 1, motor);
  /* Right side (indices 1 and 3) must carry more unsigned thrust than the left. */
  if (!(fabs(motor[1]) > fabs(motor[0]) && fabs(motor[3]) > fabs(motor[2])))
    return 1;

  mixer_compute(&cfg, 68.5, 0.0, 0.0, 0.0, 0, motor);
  if (motor[0] != 0.0 || motor[1] != 0.0 || motor[2] != 0.0 || motor[3] != 0.0)
    return 1;
  return 0;
}
