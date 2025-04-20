#ifndef POSIX_STUBS_H
#define POSIX_STUBS_H

#define ECHILD  10
#define ENOENT  2
#define EMLINK  31
#define EAGAIN  11

#define S_IFCHR 0020000

struct stat {
    int st_mode;
};

struct tms {
    clock_t tms_utime;
    clock_t tms_stime;
    clock_t tms_cutime;
    clock_t tms_cstime;
};

#endif