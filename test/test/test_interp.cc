#include <iostream>
using namespace std;

//common/numpy_fast.py--interp()
float interp(float x, float xp[], float fp[], int N)
{
    int hi = 0;

    while(hi < N && x > xp[hi]){
        hi += 1;
    }

    int low = hi -1;
    if(hi == N && x > xp[low]){
      return fp[N-1];
    }else if (hi == 0)
    {
      return fp[0];
    }else{
      return (x - xp[low]) * (fp[hi] - fp[low]) / (xp[hi] - xp[low]) + fp[low];
    }  
}


int main(int argc, char *argv[])
{
    float v_ego;
    float xp[2] = {0.0, 31.0};
    float fp[2] = {2.8, 3.5};
    for(int i = 0; i < 311; ++i){
        v_ego = i /10.0;
        cout << "v_ego = " << v_ego << "," << "interp = " << interp(v_ego, xp, fp, 2) << endl;
    }
}