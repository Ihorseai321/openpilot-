#include <time.h>
#include <sys/time.h>
#include <unistd.h>

double readclock(int clock_id)
{
    struct timespec ts;
    double current;

    clock_gettime(clock_id, &ts);
    current = ts.tv_sec + (ts.tv_nsec / 1000000000.0);
    return current;
}

double monotonic_time(){
    return readclock(CLOCK_MONOTONIC_RAW);
}

double sec_since_boot(){
    return readclock(CLOCK_BOOTTIME);
}
