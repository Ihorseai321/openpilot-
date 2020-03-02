#ifndef UTILS_H_
#define UTILS_H_

#define DT_MDL 0.05
#define MPH_TO_MS 0.44704
#define DEG_TO_RAD 0.01745329
#define RAD_TO_DEG 57.29577957

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

double readclock(int clock_id);
double monotonic_time();
double sec_since_boot();
float interp(float x, float xp[], float fp[], int n);
float min_array(float array[], int len);

#endif // UTILS_H_