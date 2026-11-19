#ifndef MAVIC_MANUAL_PID_H
#define MAVIC_MANUAL_PID_H

typedef struct {
  double kp;
  double ki;
  double kd;
  double out_min;
  double out_max;
  double i_min;
  double i_max;
  double d_tau;
  double integral;
  double prev_error;
  double d_state;
  int initialized;
} Pid;

void pid_init(Pid *pid);
void pid_configure(Pid *pid, double kp, double ki, double kd, double out_min, double out_max, double i_limit,
                   double d_tau);
void pid_reset(Pid *pid);
double pid_update(Pid *pid, double error, double dt);
int pid_self_test(void);

#endif
