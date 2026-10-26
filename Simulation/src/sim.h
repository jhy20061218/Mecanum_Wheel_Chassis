#ifndef JHY_SIM_H
#define JHY_SIM_H
#ifndef __USE_MINGW_ANSI_STDIO
#define __USE_MINGW_ANSI_STDIO 1
#endif
#include <stdio.h>
#include <stddef.h>
#include "alg_mecanum_wheel_chassis.h"
typedef struct {
    double duration, dt, a, b, radius, max_wheel_speed;
    double vx, vy, omega, kp, ki, kd, motor_tau, motor_gain, load;
    double slip, noise, ambient, initial_temperature, max_temperature;
    double feedback_timeout, max_current, print_interval, emergency_time;
    unsigned seed;
    char events[512];
} Config;
typedef struct { double time, value; char key[64]; } Event;
typedef struct { double x,y,yaw; } Pose;
typedef struct {
    Pose actual, reference, odometry;
    double rmse_position, rmse_yaw, max_position_error, max_temperature;
    double final_speed, fault_time, max_abs_wheel, max_abs_control;
    double convergence_position, convergence_yaw;
    size_t samples, saturated_samples;
    char fault[64];
} Result;
void config_default(Config *c);
int config_load(Config *c, const char *path);
int config_save(const Config *c, const char *path);
int config_validate(const Config *c);
int events_load(Event *events, size_t *count, const char *path);
int simulate(const Config *c, const Event *events, size_t count, const char *out_dir,
             int realtime, int quiet, Result *r);
int write_summary(const Config *c, const Result *r, const char *out_dir);
int edit_report(int argc, char **argv);
int self_test(void);
FILE *jhy_fopen(const char *path,const char *mode);
int jhy_remove(const char *path);
int jhy_mkdir(const char *path);
#ifndef JHY_IO_IMPLEMENTATION
#define fopen jhy_fopen
#define remove jhy_remove
#endif
#endif
