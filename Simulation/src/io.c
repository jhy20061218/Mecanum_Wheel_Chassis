/* JHY - UTF-8 filesystem paths on Windows and ordinary POSIX paths. */
#define JHY_IO_IMPLEMENTATION
#include "sim.h"
#ifdef _WIN32
#include <windows.h>
#include <direct.h>
FILE *jhy_fopen(const char *path,const char *mode) {
    wchar_t wide_path[2048],wide_mode[32];
    if(!MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,path,-1,wide_path,2048)||
       !MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,mode,-1,wide_mode,32))return NULL;
    return _wfopen(wide_path,wide_mode);
}
int jhy_remove(const char *path) {
    wchar_t wide[2048];if(!MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,path,-1,wide,2048))return -1;
    return _wremove(wide);
}
int jhy_mkdir(const char *path) {
    wchar_t wide[2048];if(!MultiByteToWideChar(CP_UTF8,MB_ERR_INVALID_CHARS,path,-1,wide,2048))return -1;
    return _wmkdir(wide);
}
#else
#include <sys/stat.h>
FILE *jhy_fopen(const char *path,const char *mode) {return fopen(path,mode);}
int jhy_remove(const char *path) {return remove(path);}
int jhy_mkdir(const char *path) {return mkdir(path,0777);}
#endif
