#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <cstdio>
#include "utils.h"

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

float clip(float x, float lo, float hi)
{
    float _min = x < hi ? x : hi;
    return lo > _min ? lo : _min;
}

float sign(float x)
{
    if(x == 0){
        return 0;
    }
    else if(x > 0){
        return 1;
    }else{
        return -1;
    }
}

float _fabs(float f)
{
    if(f < 0){
        return -f;
    }
    else{
        return f;
    }
}

void matrix_mul(int row, int col, int m,float *mat1, float *mat2, float *ret)
{
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            for (int k = 0; k < m; ++k) {
                // printf("%f ,%f\n", mat1[i * m + k], mat2[k * col + j]);
                ret[i * col + j] += (mat1[i * m + k] * mat2[k * col + j]);
            }
            // printf("%f ", ret[i * row + j]);
        }
        // printf("\n");
    }
}

void matrix_sub(int row, int col, float *A, float *B, float *ret)
{
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < col; ++j) {
            ret[i * col + j] = A[i * col + j] - B[i * col + j];
            // printf("%f ", ret[i * col + j]);
        }
        // printf("\n");
    }
}