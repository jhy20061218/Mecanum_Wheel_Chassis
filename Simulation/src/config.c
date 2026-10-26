/* JHY - strictly checked experiment configuration. */
#include "sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <errno.h>
#define FIELDS(X) X(duration) X(dt) X(a) X(b) X(radius) X(max_wheel_speed) \
 X(vx) X(vy) X(omega) X(kp) X(ki) X(kd) X(motor_tau) X(motor_gain) X(load) \
 X(slip) X(noise) X(ambient) X(initial_temperature) X(max_temperature) \
 X(feedback_timeout) X(max_current) X(print_interval) X(emergency_time)
static char *trim(char *s) {
    char *end; while(isspace((unsigned char)*s)) ++s;
    end=s+strlen(s); while(end>s && isspace((unsigned char)end[-1])) --end;
    *end='\0'; return s;
}
static int number(const char *s,double *v) {
    char *e; errno=0; *v=strtod(s,&e);
    return e!=s && *trim(e)=='\0' && errno!=ERANGE && isfinite(*v);
}
void config_default(Config *c) {
    memset(c,0,sizeof(*c));
    c->duration=12; c->dt=.01; c->a=.215; c->b=.2215; c->radius=.077;
    c->max_wheel_speed=15; c->vx=.25; c->vy=.35; c->omega=.2;
    c->kp=.08; c->ki=.6; c->kd=.0004; c->motor_tau=.12;
    c->motor_gain=20; c->load=.08; c->noise=.02;
    c->ambient=25; c->initial_temperature=25; c->max_temperature=65;
    c->feedback_timeout=.1; c->max_current=8; c->print_interval=1;
    c->emergency_time=-1; c->seed=2026;
}
static int set(Config *c,const char *key,const char *value) {
    double v;
    if(!strcmp(key,"events")) {
        if(strlen(value)>=sizeof(c->events)) return 0;
        strcpy(c->events,value); return 1;
    }
    if(!number(value,&v)) return 0;
    if(!strcmp(key,"seed")) {
        if(v<1 || v>4294967295.0 || floor(v)!=v) return 0;
        c->seed=(unsigned)v; return 1;
    }
#define SET(name) if(!strcmp(key,#name)) { c->name=v; return 1; }
    FIELDS(SET)
#undef SET
    return 0;
}
int config_validate(const Config *c) {
#define FINITE(name) if(!isfinite(c->name)) return 0;
    FIELDS(FINITE)
#undef FINITE
    return c->duration>0 && c->duration<=600 && c->dt>=.0001 && c->dt<=.05 &&
      c->duration/c->dt<=1000000 && c->a>.001 && c->a<=5 && c->b>.001 && c->b<=5 &&
      c->radius>.001 && c->radius<=1 && c->max_wheel_speed>0 && c->max_wheel_speed<=100 &&
      fabs(c->vx)<=10 && fabs(c->vy)<=10 && fabs(c->omega)<=10 &&
      c->kp>=0 && c->kp<=10 && c->ki>=0 && c->ki<=100 && c->kd>=0 && c->kd<=1 &&
      c->motor_tau>=5*c->dt && c->motor_tau<=10 &&
      c->motor_gain>0 && c->motor_gain<=200 && c->load>=0 && c->load<.9 &&
      c->slip>=0 && c->slip<1 && c->noise>=0 && c->noise<=2 &&
      c->ambient>=-20 && c->ambient<=50 && c->initial_temperature>=c->ambient &&
      c->initial_temperature<=150 && c->max_temperature>c->ambient && c->max_temperature<=120 &&
      c->feedback_timeout>=c->dt && c->feedback_timeout<=10 &&
      c->max_current>0 && c->max_current<=20 && c->print_interval>=c->dt &&
      (c->emergency_time==-1 || (c->emergency_time>=0 && c->emergency_time<c->duration)) && c->seed!=0;
}
int config_load(Config *c,const char *path) {
    FILE *f=fopen(path,"rb"); char line[1024]; size_t n=0;
    if(!f) { perror(path); return 0; }
    while(fgets(line,sizeof(line),f)) {
        char *s,*eq,*comment; ++n;
        if(!strchr(line,'\n') && !feof(f)) { fprintf(stderr,"Config line too long: %zu\n",n); fclose(f); return 0; }
        if(n==1 && (unsigned char)line[0]==0xef && (unsigned char)line[1]==0xbb && (unsigned char)line[2]==0xbf)
            memmove(line,line+3,strlen(line+3)+1);
        comment=strchr(line,'#'); if(comment) *comment='\0'; s=trim(line); if(!*s) continue;
        eq=strchr(s,'='); if(!eq) { fprintf(stderr,"Missing '=' at config line %zu\n",n); fclose(f); return 0; }
        *eq='\0'; if(!set(c,trim(s),trim(eq+1))) { fprintf(stderr,"Invalid config at line %zu: %s\n",n,s); fclose(f); return 0; }
    }
    if(ferror(f)) { fclose(f); return 0; } fclose(f);
    if(!config_validate(c)) { fputs("Config outside documented safe/numerical range.\n",stderr); return 0; }
    return 1;
}
int config_save(const Config *c,const char *path) {
    FILE *f=fopen(path,"wb"); int ok;
    if(!f) return 0;
    fputs("# JHY mecanum simulation configuration; SI units\n",f);
#define SAVE(name) fprintf(f,#name "=%.17g\n",c->name);
    FIELDS(SAVE)
#undef SAVE
    fprintf(f,"seed=%u\nevents=%s\n",c->seed,c->events);
    ok=!ferror(f); return fclose(f)==0 && ok;
}
static int event_key_valid(const char *key,double v) {
    if(!strcmp(key,"vx")||!strcmp(key,"vy")||!strcmp(key,"omega")) return fabs(v)<=10;
    if(!strcmp(key,"kp")) return v>=0&&v<=10;
    if(!strcmp(key,"ki")) return v>=0&&v<=100;
    if(!strcmp(key,"kd")) return v>=0&&v<=1;
    if(!strcmp(key,"slip")) return v>=0&&v<1;
    if(!strcmp(key,"disconnect")) return v>=0&&v<=4&&floor(v)==v;
    if(!strcmp(key,"estop")) return v==1;
    if(!strcmp(key,"temperature_LF")||!strcmp(key,"temperature_LB")||
       !strcmp(key,"temperature_RB")||!strcmp(key,"temperature_RF")) return v>=-20&&v<=150;
    return 0;
}
int events_load(Event *ev,size_t *count,const char *path) {
    FILE *f; char line[512]; size_t n=0; double prev=-1;
    *count=0; if(!path[0]) return 1; f=fopen(path,"rb"); if(!f) {perror(path);return 0;}
    while(fgets(line,sizeof(line),f)) {
        char *s,*a,*b,*c,*comment; double t,v; ++n;
        if(!strchr(line,'\n') && !feof(f)) goto bad;
        if(n==1 && strlen(line)>=3 && (unsigned char)line[0]==0xef &&
          (unsigned char)line[1]==0xbb && (unsigned char)line[2]==0xbf)
            memmove(line,line+3,strlen(line+3)+1);
        s=trim(line);
        comment=strchr(s,'#'); if(comment) *comment='\0'; s=trim(s); if(!*s) continue;
        a=strchr(s,','); if(!a) goto bad; *a++='\0'; b=strchr(a,','); if(!b) goto bad; *b++='\0';
        c=trim(a); if(*count>=256 || strlen(c)>=sizeof(ev[0].key) ||
          !number(trim(s),&t)||!number(trim(b),&v)||t<0||t<prev||!event_key_valid(c,v)) goto bad;
        ev[*count].time=t; ev[*count].value=v; strcpy(ev[*count].key,c); ++*count; prev=t;
    }
    if(ferror(f)) goto bad;
    fclose(f); return 1;
bad: fprintf(stderr,"Invalid event file at line %zu (time,key,value; ascending time).\n",n); fclose(f); return 0;
}
