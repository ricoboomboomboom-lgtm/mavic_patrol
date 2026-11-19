#include "common.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int is_bad(double value) {
  return !isfinite(value);
}

double finite_or(double value, double fallback) {
  return is_bad(value) ? fallback : value;
}

double clamp_d(double value, double low, double high) {
  if (is_bad(value))
    return 0.0;
  if (value < low)
    return low;
  if (value > high)
    return high;
  return value;
}

double wrap_pi(double angle) {
  double wrapped;

  if (is_bad(angle))
    return 0.0;

  wrapped = fmod(angle + M_PI, 2.0 * M_PI);
  if (wrapped < 0.0)
    wrapped += 2.0 * M_PI;
  return wrapped - M_PI;
}

double slew_toward(double current, double target, double max_step) {
  double delta;

  if (is_bad(current))
    current = 0.0;
  if (is_bad(target))
    return current;
  if (is_bad(max_step) || max_step < 0.0)
    return target;

  delta = target - current;
  if (delta > max_step)
    return current + max_step;
  if (delta < -max_step)
    return current - max_step;
  return target;
}

double lowpass(double current, double sample, double tau, double dt) {
  double alpha;

  if (is_bad(sample))
    return is_bad(current) ? 0.0 : current;
  if (is_bad(current) || dt <= 0.0 || tau <= 0.0)
    return sample;

  alpha = dt / (tau + dt);
  alpha = clamp_d(alpha, 0.0, 1.0);
  return current + alpha * (sample - current);
}

double apply_expo(double unit, double expo) {
  double mag;
  double shaped;
  double sign;

  unit = clamp_d(unit, -1.0, 1.0);
  expo = clamp_d(expo, 0.0, 1.0);
  sign = (unit < 0.0) ? -1.0 : 1.0;
  mag = fabs(unit);
  shaped = ((1.0 - expo) * mag) + (expo * mag * mag * mag);
  return sign * shaped;
}

int common_self_test(void) {
  int failed = 0;

  if (fabs(wrap_pi(3.0 * M_PI) - M_PI) > 1e-9 && fabs(wrap_pi(3.0 * M_PI) + M_PI) > 1e-6)
    failed = 1;
  if (fabs(wrap_pi(0.0)) > 1e-12)
    failed = 1;
  if (fabs(wrap_pi(-3.0 * M_PI / 2.0) - (M_PI / 2.0)) > 1e-9)
    failed = 1;
  if (clamp_d(NAN, -1.0, 1.0) != 0.0)
    failed = 1;
  if (fabs(slew_toward(0.0, 10.0, 1.5) - 1.5) > 1e-12)
    failed = 1;
  if (fabs(apply_expo(0.0, 0.5)) > 1e-12)
    failed = 1;
  if (apply_expo(1.0, 0.4) < 0.99)
    failed = 1;
  return failed;
}
