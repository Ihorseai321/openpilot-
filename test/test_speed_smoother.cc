#include "speed_smoother.h"
#include <iostream>
using namespace std;

int main(int argc, char const *argv[])
{
    float ret[2] = {0.0, 0.0};
    speed_smoother(0.0, 0.0, 15.833333333333334, 0.8968303686343365, -0.5803317670822143, 0.8968303686343365, -0.5803317670822143, 0.2, ret);
    cout << ret[0] << ", " << ret[1] << endl;
    return 0;
}