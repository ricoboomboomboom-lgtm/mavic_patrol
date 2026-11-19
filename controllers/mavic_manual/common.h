/*
 * Small numeric helpers shared by every module.
 * No Webots types live here, so the math can be checked without the simulator.
 */

#ifndef MAVIC_MANUAL_COMMON_H
#define MAVIC_MANUAL_COMMON_H

double clamp_d(double value, double low, double high);
double wrap_pi(double angle);
double finite_or(double value, double fallback);
int is_bad(double value);
double slew_toward(double current, double target, double max_step);
double lowpass(double current, double sample, double tau, double dt);
double apply_expo(double unit, double expo);
int common_self_test(void);

#endif
