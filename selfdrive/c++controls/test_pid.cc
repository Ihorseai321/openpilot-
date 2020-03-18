#include "lib/pid.h"

int main(int argc, char const *argv[])
{
    float kpBP[3] = {0, 0, 0};
    float kpV[3] = {0, 0, 0}; 
    float kiBP[2] = {0, 0};
    float kiV[2] = {0, 0};
    PIController p(kpBP, kpV, kiBP, kiV);
    return 0;
}