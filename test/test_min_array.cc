#include <iostream>
#include "utils.h"
using namespace std;

int main(int argc, char const *argv[])
{
    float arr[9] = {.123, 0.2345, 123., 0.1234, -1.23, 2.123, .55, .653, 123.};
    float min_all = min_array(arr, 9);
    float min_5 = min_array(arr, 5);
    cout << min_all << ", " << min_5 << endl;
    return 0;
}