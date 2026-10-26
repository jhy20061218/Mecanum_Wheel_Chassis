#include "sim.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
static int failures;
#define CHECK(x) do { if(!(x)) {fprintf(stderr,"FAIL %s:%d: %s\n",__FILE__,__LINE__,#x);++failures;} } while(0)
int self_test(void) {
    Config c;Result normal,slip,fault;Event e;Struct_Chassis_Config cfg={.215f,.2215f,.077f,15,1};
    Struct_Target_Velocity v={0,.2f,.3f,.1f};Struct_Wheel_Velocity w;double vx,vy,omega;
    failures=0;Mecanum_Wheel_Inverse_Kinematics(&cfg,&v,&w);
    vx=cfg.R*(w.LF+w.LB+w.RB+w.RF)/4;vy=cfg.R*(-w.LF+w.LB-w.RB+w.RF)/4;
    omega=cfg.R*(-w.LF-w.LB+w.RB+w.RF)/(4*(cfg.a+cfg.b));
    CHECK(fabs(vx-v.vx)<1e-6);CHECK(fabs(vy-v.vy)<1e-6);CHECK(fabs(omega-v.omega)<1e-6);
    v.vx=10;Mecanum_Wheel_Inverse_Kinematics(&cfg,&v,&w);
    CHECK(fabs(w.LF)<=15.00001&&fabs(w.LB)<=15.00001&&fabs(w.RB)<=15.00001&&fabs(w.RF)<=15.00001);
    config_default(&c);c.duration=3;c.noise=0;CHECK(config_validate(&c));
    CHECK(simulate(&c,NULL,0,NULL,0,1,&normal));CHECK(!strcmp(normal.fault,"OK"));
    CHECK(normal.rmse_position<.04);CHECK(normal.max_abs_control<=1);CHECK(isfinite(normal.actual.x));
    c.slip=.25;CHECK(simulate(&c,NULL,0,NULL,0,1,&slip));CHECK(slip.rmse_position>normal.rmse_position+.05);
    c.slip=0;e.time=1;e.value=1;strcpy(e.key,"estop");
    CHECK(simulate(&c,&e,1,NULL,0,1,&fault));CHECK(!strcmp(fault.fault,"ESTOP"));
    CHECK(fabs(fault.fault_time-1)<1e-8);CHECK(fault.final_speed<.00001);
    strcpy(e.key,"disconnect");CHECK(simulate(&c,&e,1,NULL,0,1,&fault));
    CHECK(!strcmp(fault.fault,"FEEDBACK_TIMEOUT_LF"));CHECK(fault.fault_time>=1&&fault.fault_time<=1.11);
    strcpy(e.key,"temperature_RF");e.value=90;CHECK(simulate(&c,&e,1,NULL,0,1,&fault));
    CHECK(!strcmp(fault.fault,"OVERHEAT_RF"));CHECK(fault.final_speed<.00001);
    c.radius=0;CHECK(!config_validate(&c));config_default(&c);c.dt=NAN;CHECK(!config_validate(&c));
    printf("Self-test: %s (%d failure(s)); kinematics, limits, normal/slip, estop, timeout, overheat, invalid config.\n",failures?"FAIL":"PASS",failures);
    return failures==0;
}
