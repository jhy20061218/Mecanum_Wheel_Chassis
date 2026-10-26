/* JHY - portable C11 console experiment entry point. */
#include "sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#ifdef _WIN32
#include <windows.h>
#include <shellapi.h>
#endif
static void usage(void) {
    puts("JHY Mecanum prediction & simulation (C11)\n"
      "  mecanum_sim --config config/default.cfg --output results/default [--realtime] [--quiet]\n"
      "  mecanum_sim --self-test\n"
      "  mecanum_sim --write-config FILE\n"
      "  mecanum_sim --edit-report FILE append TEXT\n"
      "  mecanum_sim --edit-report FILE delete TEXT\n"
      "  mecanum_sim --edit-report FILE replace OLD NEW\n"
      "  Use @utf8_file as TEXT/OLD/NEW to read UTF-8 text from a file.\n"
      "  No hardware is accessed. Output is software simulation, not physical test data.");
}
static int copy_events(const Event *events,size_t count,const char *dir) {
    char path[1024]; FILE *f; size_t i; int ok;
    snprintf(path,sizeof(path),"%s/used_events.csv",dir); f=fopen(path,"wb"); if(!f) return 0;
    for(i=0;i<count;++i) fprintf(f,"%.17g,%s,%.17g\n",events[i].time,events[i].key,events[i].value);
    ok=!ferror(f);return fclose(f)==0&&ok;
}
static int run(int argc,char **argv) {
    Config c,coarse_config,fine_config; Result r,coarse,fine; Event events[256]; size_t count=0,i;
    const char *config=NULL,*dir="results/default"; int realtime=0,quiet=0;
    char path[1024]; config_default(&c);
    if(argc==1 || (argc==2&&!strcmp(argv[1],"--help"))) {usage();return 0;}
    if(argc==2&&!strcmp(argv[1],"--self-test")) return self_test()?0:1;
    if(argc==3&&!strcmp(argv[1],"--write-config")) return config_save(&c,argv[2])?0:1;
    if(argc>=2&&!strcmp(argv[1],"--edit-report")) return edit_report(argc,argv)?0:1;
    for(i=1;i<(size_t)argc;++i) {
        if(!strcmp(argv[i],"--config")&&i+1<(size_t)argc) config=argv[++i];
        else if(!strcmp(argv[i],"--output")&&i+1<(size_t)argc) dir=argv[++i];
        else if(!strcmp(argv[i],"--realtime")) realtime=1;
        else if(!strcmp(argv[i],"--quiet")) quiet=1;
        else {fprintf(stderr,"Unknown/incomplete option: %s\n",argv[i]);usage();return 1;}
    }
    if(config&&!config_load(&c,config)) return 1;
    if(!config_validate(&c)||!events_load(events,&count,c.events)) return 1;
    for(i=0;i<count;++i) if(events[i].time>=c.duration) {fputs("Event time must be before experiment end.\n",stderr);return 1;}
    if(!simulate(&c,events,count,dir,realtime,quiet,&r)) return 1;
    /* Numerical sensitivity is separated from changes in random sampling density. */
    coarse_config=c;coarse_config.noise=0;fine_config=coarse_config;fine_config.dt=c.dt/2;
    if(!simulate(&coarse_config,events,count,NULL,0,1,&coarse)||
       !simulate(&fine_config,events,count,NULL,0,1,&fine)) return 1;
    r.convergence_position=hypot(coarse.actual.x-fine.actual.x,coarse.actual.y-fine.actual.y);
    r.convergence_yaw=fabs(coarse.actual.yaw-fine.actual.yaw);
    snprintf(path,sizeof(path),"%s/used_config.cfg",dir);
    if(!config_save(&c,path)||!copy_events(events,count,dir)||!write_summary(&c,&r,dir)) {
        fputs("Cannot write complete output report.\n",stderr);return 1;
    }
    printf("Result: status=%s samples=%zu position_RMSE=%.8fm yaw_RMSE=%.8frad dt/2_position_difference=%.9fm\nOutput: %s\n",r.fault,r.samples,r.rmse_position,r.rmse_yaw,r.convergence_position,dir);
    return 0;
}
int main(int argc,char **argv) {
#ifdef _WIN32
    /* Windows CRT narrow argv can lose Chinese text; decode native UTF-16 argv. */
    int wide_count=0,i,result; LPWSTR *wide=CommandLineToArgvW(GetCommandLineW(),&wide_count);
    char **utf8;
    SetConsoleOutputCP(CP_UTF8);
    if(!wide) return 1;
    utf8=(char **)calloc((size_t)wide_count+1,sizeof(char *)); if(!utf8) {LocalFree(wide);return 1;}
    for(i=0;i<wide_count;++i) {
        int n=WideCharToMultiByte(CP_UTF8,0,wide[i],-1,NULL,0,NULL,NULL);
        utf8[i]=(char *)malloc((size_t)n);
        if(!utf8[i]) {while(i>0) free(utf8[--i]);free(utf8);LocalFree(wide);return 1;}
        WideCharToMultiByte(CP_UTF8,0,wide[i],-1,utf8[i],n,NULL,NULL);
    }
    (void)argc;(void)argv;result=run(wide_count,utf8);
    for(i=0;i<wide_count;++i) free(utf8[i]);
    free(utf8);LocalFree(wide);return result;
#else
    return run(argc,argv);
#endif
}
