#include "vehicle_model.h"
#include <iostream>

using namespace std;

int main(int argc, char const *argv[])
{
    VehicleModel VM;
    cout << VM.yaw_rate(20 * 3.1415926535898 / 180, 10.0) << endl;
    return 0;
}