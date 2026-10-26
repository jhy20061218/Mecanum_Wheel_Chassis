/* JHY - software-only experiment; never accesses motor/CAN hardware. */
#define _POSIX_C_SOURCE 200809L
#include "sim.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#ifdef _WIN32
#include <windows.h>
#include <direct.h>
#else
#include <time.h>
#include <sys/stat.h>
#endif
static const char *wheel_name[4]={"LF","LB","RB","RF"};
static double clamp(double x,double lo,double hi) { return x<lo?lo:x>hi?hi:x; }
static double random_unit(uint32_t *state) {
    *state=*state*1664525u+1013904223u;
    return 2.0*((double)*state/4294967295.0)-1.0;
}
static void delay_seconds(double sec) {
#ifdef _WIN32
    Sleep((DWORD)(sec*1000+.5));
#else
    struct timespec ts; ts.tv_sec=(time_t)sec; ts.tv_nsec=(long)((sec-(double)ts.tv_sec)*1e9);
    nanosleep(&ts,NULL);
#endif
}
static int make_directory(const char *p) {
    FILE *test; char file[1024],parent[1024];size_t k;
    if(strlen(p)>=sizeof(parent))return 0;
    strcpy(parent,p);
    for(k=1;parent[k];++k) if(parent[k]=='/'||parent[k]=='\\') {
        char sep=parent[k];parent[k]='\0';
        if(k>2 || parent[1]!=':') (void)jhy_mkdir(parent);
        parent[k]=sep;
    }
    (void)jhy_mkdir(p);
    if(snprintf(file,sizeof(file),"%s/.write_check",p)<0) return 0;
    test=fopen(file,"wb"); if(!test) {perror(p);return 0;}
    if(fclose(test)!=0) return 0;
    return remove(file)==0;
}
static void forward(const Config *c,const double w[4],double *vx,double *vy,double *omega) {
    *vx=c->radius*(w[0]+w[1]+w[2]+w[3])/4;
    *vy=c->radius*(-w[0]+w[1]-w[2]+w[3])/4;
    *omega=c->radius*(-w[0]-w[1]+w[2]+w[3])/(4*(c->a+c->b));
}
static void integrate(Pose *p,double vx,double vy,double omega,double dt) {
    /* Midpoint heading integrates the constant body twist to second order. */
    double angle=p->yaw+.5*omega*dt;
    p->x+=(cos(angle)*vx-sin(angle)*vy)*dt;
    p->y+=(sin(angle)*vx+cos(angle)*vy)*dt; p->yaw+=omega*dt;
}
static void targets(const Config *c,double w[4],int *saturated) {
    Struct_Chassis_Config cfg={(float)c->a,(float)c->b,(float)c->radius,(float)c->max_wheel_speed,1.0f};
    Struct_Target_Velocity target={0,(float)c->vx,(float)c->vy,(float)c->omega};
    Struct_Wheel_Velocity out;
    double raw=(fabs(c->vx)+fabs(c->vy)+fabs(c->omega)*(c->a+c->b))/c->radius;
    *saturated=raw>c->max_wheel_speed;
    /* Exactly the same source function is linked by PC simulation and STM32. */
    Mecanum_Wheel_Inverse_Kinematics(&cfg,&target,&out);
    w[0]=out.LF;w[1]=out.LB;w[2]=out.RB;w[3]=out.RF;
}
static void latch(Result *r,const char *reason,double time) {
    if(!r->fault[0]) {snprintf(r->fault,sizeof(r->fault),"%s",reason);r->fault_time=time;}
}
static void apply_event(Config *c,const Event *e,int online[4],double temp[4],Result *r,double time) {
    if(!strcmp(e->key,"vx")) c->vx=e->value;
    else if(!strcmp(e->key,"vy")) c->vy=e->value;
    else if(!strcmp(e->key,"omega")) c->omega=e->value;
    else if(!strcmp(e->key,"kp")) c->kp=e->value;
    else if(!strcmp(e->key,"ki")) c->ki=e->value;
    else if(!strcmp(e->key,"kd")) c->kd=e->value;
    else if(!strcmp(e->key,"slip")) c->slip=e->value;
    else if(!strcmp(e->key,"disconnect")) {
        int i; for(i=0;i<4;++i) online[i]=e->value==0 || (int)e->value!=i+1;
    } else if(!strcmp(e->key,"estop")) latch(r,"ESTOP",time);
    else if(!strncmp(e->key,"temperature_",12)) {
        int i; for(i=0;i<4;++i) if(!strcmp(e->key+12,wheel_name[i])) temp[i]=e->value;
    }
}
int simulate(const Config *original,const Event *events,size_t count,const char *out_dir,
             int realtime,int quiet,Result *r) {
    Config c=*original; FILE *csv=NULL; char path[1024];
    double w[4]={0},measured[4]={0},integral[4]={0},previous[4]={0},temperature[4];
    double age[4]={0},u[4]={0},current[4]={0},target[4],effective[4];
    int online[4]={1,1,1,1},i; size_t step=0,index=0;
    double time=0,next_print=0,sum_pos=0,sum_yaw=0; uint32_t rng=c.seed;
    memset(r,0,sizeof(*r)); r->fault_time=-1;
    for(i=0;i<4;++i) temperature[i]=c.initial_temperature;
    r->max_temperature=c.initial_temperature;
    if(out_dir) {
        if(strlen(out_dir)>800 || !make_directory(out_dir)) return 0;
        snprintf(path,sizeof(path),"%s/telemetry.csv",out_dir); csv=fopen(path,"wb"); if(!csv) return 0;
        fputs("time_s,vx_command_mps,vy_command_mps,omega_command_radps,x_reference_m,y_reference_m,yaw_reference_rad,x_actual_m,y_actual_m,yaw_actual_rad,x_odometry_m,y_odometry_m,yaw_odometry_rad,position_error_m,yaw_error_rad,slip,limited,fault",csv);
        for(i=0;i<4;++i) fprintf(csv,",%s_target_radps,%s_actual_radps,%s_measured_radps,%s_control,%s_current_est_A,%s_temperature_C,%s_feedback_age_s,%s_online",wheel_name[i],wheel_name[i],wheel_name[i],wheel_name[i],wheel_name[i],wheel_name[i],wheel_name[i],wheel_name[i]);
        fputc('\n',csv);
    }
    while(time<c.duration-1e-12) {
        double h=fmin(c.dt,c.duration-time),vx,vy,omega,error_pos,error_yaw; int saturated;
        while(index<count && events[index].time<=time+1e-10) {
            apply_event(&c,&events[index],online,temperature,r,time); ++index;
        }
        if(c.emergency_time>=0 && time+1e-10>=c.emergency_time) latch(r,"ESTOP",time);
        targets(&c,target,&saturated); if(saturated) ++r->saturated_samples;
        for(i=0;i<4;++i) {
            char reason[64];
            if(temperature[i]>r->max_temperature) r->max_temperature=temperature[i];
            if(online[i]) { age[i]=0; measured[i]=w[i]+c.noise*random_unit(&rng); }
            if(age[i]+1e-10>=c.feedback_timeout) {snprintf(reason,sizeof(reason),"FEEDBACK_TIMEOUT_%s",wheel_name[i]);latch(r,reason,time);}
            if(temperature[i]>=c.max_temperature) {snprintf(reason,sizeof(reason),"OVERHEAT_%s",wheel_name[i]);latch(r,reason,time);}
            if(fabs(w[i])>1.2*c.max_wheel_speed) latch(r,"OVERSPEED",time);
            if(!isfinite(w[i]) || !isfinite(measured[i]) || !isfinite(temperature[i])) latch(r,"NONFINITE",time);
        }
        for(i=0;i<4;++i) {
            double error=target[i]-measured[i],candidate,raw;
            if(r->fault[0]) {u[i]=0;integral[i]=0;}
            else {
                candidate=clamp(integral[i]+error*h,-1,1);
                /* A normalized drive command; feedforward is a steady velocity term. */
                raw=target[i]/c.motor_gain+c.kp*error+c.ki*candidate-c.kd*(measured[i]-previous[i])/h;
                u[i]=clamp(raw,-1,1);
                /* Conditional integration prevents further windup into saturation. */
                if(fabs(raw)<=1 || (raw>1 && error<0) || (raw< -1 && error>0)) integral[i]=candidate;
            }
            previous[i]=measured[i]; current[i]=fabs(u[i])*c.max_current;
            w[i]+=h*(c.motor_gain*(1-c.load)*u[i]-w[i])/c.motor_tau;
            temperature[i]+=h*(.025*current[i]*current[i]-.08*(temperature[i]-c.ambient));
            effective[i]=w[i]*(1-c.slip);
            if(temperature[i]>r->max_temperature) r->max_temperature=temperature[i];
            if(fabs(w[i])>r->max_abs_wheel) r->max_abs_wheel=fabs(w[i]);
            if(fabs(u[i])>r->max_abs_control) r->max_abs_control=fabs(u[i]);
            if(!online[i]) age[i]+=h;
        }
        forward(&c,effective,&vx,&vy,&omega); integrate(&r->actual,vx,vy,omega,h);
        forward(&c,target,&vx,&vy,&omega); integrate(&r->reference,vx,vy,omega,h);
        forward(&c,measured,&vx,&vy,&omega); integrate(&r->odometry,vx,vy,omega,h);
        time+=h; ++step;
        error_pos=hypot(r->actual.x-r->reference.x,r->actual.y-r->reference.y);
        error_yaw=r->actual.yaw-r->reference.yaw;
        sum_pos+=error_pos*error_pos*h; sum_yaw+=error_yaw*error_yaw*h;
        if(error_pos>r->max_position_error) r->max_position_error=error_pos;
        if(csv) {
            fprintf(csv,"%.9f,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%s",time,c.vx,c.vy,c.omega,r->reference.x,r->reference.y,r->reference.yaw,r->actual.x,r->actual.y,r->actual.yaw,r->odometry.x,r->odometry.y,r->odometry.yaw,error_pos,error_yaw,c.slip,saturated,r->fault[0]?r->fault:"OK");
            for(i=0;i<4;++i) fprintf(csv,",%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d",r->fault[0]?0:target[i],w[i],measured[i],u[i],current[i],temperature[i],age[i],online[i]);
            fputc('\n',csv);
        }
        if(!quiet && (time+1e-10>=next_print || time+1e-10>=c.duration)) {
            printf("t=%6.2fs pose=(%7.3f,%7.3f,%7.3f) error=%.4fm wheels=[%.2f %.2f %.2f %.2f] T=[%.1f %.1f %.1f %.1f] status=%s\n",time,r->actual.x,r->actual.y,r->actual.yaw,error_pos,w[0],w[1],w[2],w[3],temperature[0],temperature[1],temperature[2],temperature[3],r->fault[0]?r->fault:"OK");
            next_print=time+c.print_interval;
        }
        if(realtime) delay_seconds(h);
    }
    r->samples=step; r->rmse_position=sqrt(sum_pos/c.duration); r->rmse_yaw=sqrt(sum_yaw/c.duration);
    r->final_speed=0; for(i=0;i<4;++i) r->final_speed=fmax(r->final_speed,fabs(w[i]));
    if(csv) {int ok=!ferror(csv); if(fclose(csv)!=0 || !ok) {fputs("Telemetry write failed.\n",stderr);return 0;}}
    if(!r->fault[0]) strcpy(r->fault,"OK");
    return 1;
}
int write_summary(const Config *c,const Result *r,const char *dir) {
    FILE *f; char path[1024]; int ok;
    snprintf(path,sizeof(path),"%s/summary.json",dir); f=fopen(path,"wb"); if(!f) return 0;
    fprintf(f,"{\n  \"author\": \"JHY\",\n  \"data_source\": \"software_simulation\",\n  \"duration_s\": %.9g,\n  \"dt_s\": %.9g,\n  \"seed\": %u,\n  \"samples\": %zu,\n  \"rmse_position_m\": %.12g,\n  \"rmse_yaw_rad\": %.12g,\n  \"max_position_error_m\": %.12g,\n  \"maximum_temperature_C\": %.12g,\n  \"maximum_wheel_speed_radps\": %.12g,\n  \"maximum_control\": %.12g,\n  \"final_max_wheel_speed_radps\": %.12g,\n  \"saturated_samples\": %zu,\n  \"fault\": \"%s\",\n  \"fault_time_s\": %.9g,\n  \"dt_halving_position_difference_m\": %.12g,\n  \"dt_halving_yaw_difference_rad\": %.12g,\n  \"actual_final\": [%.12g,%.12g,%.12g],\n  \"reference_final\": [%.12g,%.12g,%.12g],\n  \"odometry_final\": [%.12g,%.12g,%.12g]\n}\n",c->duration,c->dt,c->seed,r->samples,r->rmse_position,r->rmse_yaw,r->max_position_error,r->max_temperature,r->max_abs_wheel,r->max_abs_control,r->final_speed,r->saturated_samples,r->fault,r->fault_time,r->convergence_position,r->convergence_yaw,r->actual.x,r->actual.y,r->actual.yaw,r->reference.x,r->reference.y,r->reference.yaw,r->odometry.x,r->odometry.y,r->odometry.yaw);
    ok=!ferror(f); if(fclose(f)!=0 || !ok) return 0;
    snprintf(path,sizeof(path),"%s/report.md",dir); f=fopen(path,"wb"); if(!f) return 0;
    fprintf(f,"# 麦克纳姆轮底盘预测与仿真实验报告\n\n署名：JHY\n\n## 实验目的与数据性质\n\n通过共享原工程逆运动学源码，研究速度闭环、负载、滑移、反馈故障及急停对底盘轨迹的影响。本次数据来自软件仿真，未连接实物，不作为真实硬件实验记录。\n\n## 方案与参数\n\n轮序 LF、LB、RB、RF；x 向右，y 向前，偏航按原工程代数符号。几何半宽 a=%.4f m、半长 b=%.4f m、轮半径 R=%.4f m。仿真时长 %.3f s，步长 %.6f s，初始指令 (%.3f, %.3f, %.3f)，随机种子 %u。\n\n初始 PID 参数 Kp=%.6f、Ki=%.6f、Kd=%.6f；一阶电机时间常数 %.4f s、增益 %.3f rad/s、负载折减率 %.3f、滑移率 %.3f；测量噪声为 [-%.3f, %.3f] rad/s 均匀分布。事件文件：%s。完整初始配置见 used_config.cfg。\n\n## 模型与算法\n\n逆运动学直接调用 JHY/Src/alg_mecanum_wheel_chassis.c，并等比例限幅。速度环采用稳态前馈和带条件积分防饱和的 PID，输出 u 限制在 [-1,1]。\n\n电机模型：dω/dt=[K(1-load)u-ω]/τ，显式欧拉离散；有效地面轮速为 (1-slip)ω；编码器测量包含均匀噪声。正运动学计算底盘速度，中点航向积分位姿。温度模型 dT/dt=0.025 I²-0.08(T-Ta)，I=|u|Imax，仅作教学估算。\n\n理想参考由限幅后的目标轮速积分；实际轨迹由电机模型与滑移计算；里程计轨迹由有噪声的编码器速度计算。三者独立记录，以展示滑移下里程计的局限。\n\n## 结果\n\n| 指标 | 数值 |\n|---|---:|\n| 采样行数 | %zu |\n| 位置 RMSE / m | %.8f |\n| 航向 RMSE / rad | %.8f |\n| 最大位置误差 / m | %.8f |\n| 最高模拟温度 / °C | %.4f |\n| 最大轮速 / rad/s | %.6f |\n| 最大驱动指令绝对值 | %.6f |\n| 目标限幅次数 | %zu |\n| 故障状态 | %s |\n| 首次故障时刻 / s | %.6f |\n| 步长减半终点位置差 / m | %.9f |\n| 步长减半终点航向差 / rad | %.9f |\n\n最终实际位姿：(%.8f m, %.8f m, %.8f rad)。最终理想参考位姿：(%.8f m, %.8f m, %.8f rad)。\n\n## 准确性、可靠性与局限\n\n1. RMSE 是模型输出相对理想参考的跟踪误差，不能解释为相对实物真值的精度。未经过实测标定，不能给出实车精度保证。\n2. 步长减半比较关闭随机噪声，其差值衡量离散误差敏感性；事件按控制步开始量化执行，执行时刻误差小于一个步长。单次比较不等同于完整收敛阶验证。\n3. 一阶电机、均匀滑移、温度与电流估算未描述载荷转移、轮胎滚子碰撞、地面非均匀摩擦、电池动态、CAN 仲裁与采样抖动。热阈值为课程实验参数，不能替代厂家保护设定。\n4. 固定随机种子保证可复现，但单次噪声实验不构成统计置信区间。应在实物上进行编码器、轴向符号和几何参数标定，并用独立定位系统评估误差。\n5. 软件检测过温、反馈超时、超速、非有限值和急停，故障锁存后四路驱动输出为零，轮速按模型衰减；真实硬件仍须配置独立急停、机械防护与断电措施。\n\n## 数据与复现\n\ntelemetry.csv 为每步原始仿真记录，summary.json 为结构化指标，used_config.cfg 与 used_events.csv 保存本次初始输入。控制台按时间间隔输出四轮速度、温度和全局状态；--realtime 为近似墙钟演示节拍，不是硬实时保证。\n\n## 工作记录\n\n完成模型构建、参数检查、闭环计算、状态监测、CSV 记录和报告生成。可通过 --edit-report 在命令行追加、删除或替换报告文本；编辑后应自行核对结论与数据一致性。\n",c->a,c->b,c->radius,c->duration,c->dt,c->vx,c->vy,c->omega,c->seed,c->kp,c->ki,c->kd,c->motor_tau,c->motor_gain,c->load,c->slip,c->noise,c->noise,c->events[0]?c->events:"无",r->samples,r->rmse_position,r->rmse_yaw,r->max_position_error,r->max_temperature,r->max_abs_wheel,r->max_abs_control,r->saturated_samples,r->fault,r->fault_time,r->convergence_position,r->convergence_yaw,r->actual.x,r->actual.y,r->actual.yaw,r->reference.x,r->reference.y,r->reference.yaw);
    fputs("\n## 数据时序补充\n\n参考轨迹在故障后仍按持续任务指令积分；CSV 四轮 target_radps 为联锁后的施加目标，故障时归零，而三轴 command 字段仍保留用户任务。故障场景的轨迹误差反映偏离原计划，停机保护应通过四路 control=0 及实际轮速衰减验证。\n\nCSV time_s 为控制步结束时刻；measured 是步初采样，actual、位姿、温度为步末模型状态，fault_time_s 是检测控制步的开始时刻。yaw_error_rad 与航向 RMSE 使用未折回角度区间的累计航向差。\n",f);
    ok=!ferror(f); return fclose(f)==0 && ok;
}
