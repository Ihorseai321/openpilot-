#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <cstdio>

struct LeadData {
    float dRel;
    float yRel;
    float vRel;
    float aRel;
    float vLead;
    float aLeadDEPRECATED;
    float dPath;
    float vLat;
    float vLeadK;
    float aLeadK;
    bool fcw;
    bool status;
    float aLeadTau;
    float modelProb;
    bool radar;
};

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

float interp(float x, float xp[], float fp[], int n)
{
    int hi = 0;

    while(hi < n && x > xp[hi]){
        hi += 1;
    }

    int low = hi -1;
    if(hi == n && x > xp[low]){
      return fp[n-1];
    }else if (hi == 0)
    {
      return fp[0];
    }else{
      return (x - xp[low]) * (fp[hi] - fp[low]) / (xp[hi] - xp[low]) + fp[low];
    }  
}

float min_array(float array[], int len)
{
    float min_val = 10000.0;
    float min_total = 10000.1;
    // int c_i;
    // int c_t;
    for(int i = 0; i < len; ++i){
        if(i <= len -1 - i){
            min_val = array[i] < array[len-i-1] ? array[i] : array[len-i-1];
            // c_i = array[i] < array[len-i] ? i : len-i-1;
            min_total = min_total < min_val ? min_total : min_val;
            // c_t = min_total < min_val ? c_t : c_i;
        }
    }
    // printf("%d\n", c_t);
    return min_total;
}