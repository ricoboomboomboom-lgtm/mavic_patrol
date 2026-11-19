#include "pid.h"

#include "common.h"

#include <math.h>
#include <stddef.h>

void pid_init(Pid *pid) {
  if (!pid)
    return;
  pid->kp = 0.0;
  pid->ki = 0.0;
  pid->kd = 0.0;
  pid->out_min = -1.0;
  pid->out_max = 1.0;
  pid->i_min = -1.0;
  pid->i_max = 1.0;
  pid->d_tau = 0.0;
  pid_reset(pid);
}

void pid_configure(Pid *pid, double kp, double ki, double kd, double out_min, double out_max, double i_limit,
                   double d_tau) {
  if (!pid)
    return;
  pid->kp = finite_or(kp, 0.0);
  pid->ki = finite_or(ki, 0.0);
  pid->kd = finite_or(kd, 0.0);
  pid->out_min = finite_or(out_min, -1.0);
  pid->out_max = finite_or(out_max, 1.0);
  if (pid->out_max < pid->out_min) {
    double swap = pid->out_min;
    pid->out_min = pid->out_max;
    pid->out_max = swap;
  }
  i_limit = fabs(finite_or(i_limit, 0.0));
  pid->i_min = -i_limit;
  pid->i_max = i_limit;
  pid->d_tau = finite_or(d_tau, 0.0);
  if (pid->d_tau < 0.0)
    pid->d_tau = 0.0;
}

void pid_reset(Pid *pid) {
  if (!pid)
    return;
  pid->integral = 0.0;
  pid->prev_error = 0.0;
  pid->d_state = 0.0;
  pid->initialized = 0;
}

double pid_update(Pid *pid, double error, double dt) {
  double derivative;
  double unsaturated;
  double saturated;
  int allow_integrate;

  if (!pid || is_bad(error) || is_bad(dt) || dt <= 0.0 || dt > 0.1)
    return 0.0;

  if (!pid->initialized) {
    pid->prev_error = error;
    pid->d_state = 0.0;
    pid->initialized = 1;
    derivative = 0.0;
  } else {
    derivative = (error - pid->prev_error) / dt;
    if (pid->d_tau > 0.0)
      derivative = lowpass(pid->d_state, derivative, pid->d_tau, dt);
    pid->d_state = derivative;
    pid->prev_error = error;
  }

  if (is_bad(derivative))
    derivative = 0.0;

  unsaturated = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
  saturated = clamp_d(unsaturated, pid->out_min, pid->out_max);

  /*
   * Conditional integration: freeze the integrator when the output is
   * already hard against a limit and the error would push it further.
   */
  allow_integrate = 1;
  if (saturated >= pid->out_max && error > 0.0)
    allow_integrate = 0;
  if (saturated <= pid->out_min && error < 0.0)
    allow_integrate = 0;
  if (allow_integrate && pid->ki != 0.0)
    pid->integral += error * dt;
  pid->integral = clamp_d(pid->integral, pid->i_min, pid->i_max);
  if (is_bad(pid->integral))
    pid->integral = 0.0;

  unsaturated = (pid->kp * error) + (pid->ki * pid->integral) + (pid->kd * derivative);
  return clamp_d(unsaturated, pid->out_min, pid->out_max);
}

int pid_self_test(void) {
  Pid pid;
  double output;
  int i;

  pid_init(&pid);
  pid_configure(&pid, 1.0, 0.5, 0.0, -0.5, 0.5, 10.0, 0.0);
  output = 0.0;
  for (i = 0; i < 100; ++i)
    output = pid_update(&pid, 1.0, 0.008);
  if (output > 0.5 + 1e-9 || output < 0.0)
    return 1;
  if (pid.integral > 10.0 + 1e-9)
    return 1;
  if (pid_update(&pid, NAN, 0.008) != 0.0)
    return 1;
  if (pid_update(NULL, 1.0, 0.008) != 0.0)
    return 1;
  return 0;
}
