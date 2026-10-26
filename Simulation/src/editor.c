/* JHY - byte-preserving UTF-8 report editing with atomic replacement. */
#include "sim.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifdef _WIN32
#include <windows.h>
#endif
static char *load_text(const char *path) {
    FILE *f=fopen(path,"rb"); long n; char *s;
    if(!f) return NULL;
    if(fseek(f,0,SEEK_END)!=0 || (n=ftell(f))<0 || n>8*1024*1024 || fseek(f,0,SEEK_SET)!=0) {fclose(f);return NULL;}
    s=(char *)malloc((size_t)n+1);if(!s) {fclose(f);return NULL;}
    {int read_ok=fread(s,1,(size_t)n,f)==(size_t)n;
     if(fclose(f)!=0||!read_ok) {free(s);return NULL;}}
    s[n]='\0';if(strlen(s)!=(size_t)n) {free(s);return NULL;} return s;
}
static int valid_utf8(const unsigned char *s) {
    while(*s) {
        unsigned cp;int n,i;
        if(*s<128) {++s;continue;}
        if(*s>=0xc2&&*s<=0xdf) {n=1;cp=*s&31;}
        else if(*s>=0xe0&&*s<=0xef) {n=2;cp=*s&15;}
        else if(*s>=0xf0&&*s<=0xf4) {n=3;cp=*s&7;}
        else return 0;
        ++s;for(i=0;i<n;++i) {if((*s&0xc0)!=0x80)return 0;cp=(cp<<6)|(*s++&63);}
        if((n==2&&cp<0x800)||(n==3&&cp<0x10000)||cp>0x10ffff||(cp>=0xd800&&cp<=0xdfff))return 0;
    }return 1;
}
static char *argument_text(const char *s) {
    char *copy; if(s[0]=='@') return load_text(s+1);
    copy=(char *)malloc(strlen(s)+1); if(copy) strcpy(copy,s);return copy;
}
int edit_report(int argc,char **argv) {
    const char *file,*mode;char *original=NULL,*old=NULL,*replacement=NULL,*output=NULL,*cursor,*match;
    char temp[1024];size_t capacity,out=0,old_n,new_n,changes=0;FILE *f;int ok=0;
    if(argc<5||argc>6) goto done;
    file=argv[2];mode=argv[3];
    if(strcmp(mode,"append")&&strcmp(mode,"delete")&&strcmp(mode,"replace")) goto done;
    if((!strcmp(mode,"replace")&&argc!=6)||(strcmp(mode,"replace")&&argc!=5))goto done;
    original=load_text(file);old=argument_text(argv[4]);
    replacement=argument_text(!strcmp(mode,"replace")?argv[5]:"");
    if(!original||!old||!replacement||!valid_utf8((unsigned char *)original)||
       !valid_utf8((unsigned char *)old)||!valid_utf8((unsigned char *)replacement)) goto done;
    old_n=strlen(old);new_n=strlen(replacement);
    if(!old_n) goto done;
    if(!strcmp(mode,"append")) {
        capacity=strlen(original)+old_n+3;output=(char *)malloc(capacity);if(!output)goto done;
        snprintf(output,capacity,"%s\n%s\n",original,old);changes=1;
    } else {
        size_t occurrences=0;cursor=original;
        while((match=strstr(cursor,old))!=NULL) {++occurrences;cursor=match+old_n;}
        if(!occurrences) {fputs("No matching text; report unchanged.\n",stderr);goto done;}
        capacity=strlen(original)+1;
        if(new_n>old_n) {
            if(occurrences>(16u*1024u*1024u-capacity)/(new_n-old_n))goto done;
            capacity+=occurrences*(new_n-old_n);
        }
        output=(char *)malloc(capacity);if(!output)goto done;cursor=original;
        while((match=strstr(cursor,old))!=NULL) {
            size_t n=(size_t)(match-cursor);memcpy(output+out,cursor,n);out+=n;
            memcpy(output+out,replacement,new_n);out+=new_n;cursor=match+old_n;++changes;
        }strcpy(output+out,cursor);
    }
    if(strlen(file)>900)goto done;
    snprintf(temp,sizeof(temp),"%s.jhy-edit.tmp",file);
    f=fopen(temp,"wb");if(!f)goto done;
    {size_t n=strlen(output);int wrote=fwrite(output,1,n,f)==n;if(fclose(f)!=0||!wrote){remove(temp);goto done;}}
#ifdef _WIN32
    {wchar_t wide_from[1024],wide_to[1024];
     if(!MultiByteToWideChar(CP_UTF8,0,temp,-1,wide_from,1024)||
        !MultiByteToWideChar(CP_UTF8,0,file,-1,wide_to,1024)||
        !MoveFileExW(wide_from,wide_to,MOVEFILE_REPLACE_EXISTING|MOVEFILE_WRITE_THROUGH)) {remove(temp);goto done;}}
#else
    if(rename(temp,file)!=0){remove(temp);goto done;}
#endif
    printf("Edited report: %zu change(s).\n",changes);ok=1;
done:free(original);free(old);free(replacement);free(output);
    if(!ok)fputs("Report edit failed (use valid UTF-8 text, existing report, and append/delete/replace).\n",stderr);
    return ok;
}
