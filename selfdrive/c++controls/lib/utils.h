#ifndef UTILS_H_
#define UTILS_H_

#define DT_MDL 0.05
#define DT_CTRL 0.01
#define MPH_TO_MS 0.44704
#define MS_TO_KPH 3.6
#define MS_TO_MPH 2.23693629
#define DEG_TO_RAD 0.01745329
#define RAD_TO_DEG 57.29577957
#define CAMERA_OFFSET 0.06

#define ARRAYSIZE(x) (sizeof(x)/sizeof(x[0]))


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

typedef struct
{
    float final_gas;
    float final_brake;
}LCtrlRet;

double readclock(int clock_id);
double monotonic_time();
double sec_since_boot();
float interp(float x, float xp[], float fp[], int n);
float min_array(float array[], int len);
float clip(float x, float lo, float hi);
float sign(float x);
float _fabs(float f);
void matrix_mul(int row, int col, int m,float *mat1, float *mat2, float *result);
void matrix_sub(int row, int col, float *A, float *B, float *ret);
#endif // UTILS_H_