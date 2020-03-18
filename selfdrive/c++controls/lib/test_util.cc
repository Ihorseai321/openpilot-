#include "utils.h"
#include <string.h>
#include <iostream>
using namespace std;

class test_util
{
public:
  test_util()
  {
    for(int i = 0; i < 12; ++i){
      arr[i] = i * 1.0156;
    }
  }
  
  float arr[12]; 
};

int main(int argc, char const *argv[])
{
  float x[3][2] = {{1, 2}, {3, 4}, {5, 6}};
  float _x[3][2] = {{1, 1}, {1, 1}, {1, 1}};
  float _ret[3][2] = {{1, 1}, {1, 1}, {1, 1}};
  float y[2][3] = {{1, 2, 3}, {4, 5, 6}};
  float ret[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
  matrix_mul(3, 3, 2, x[0], y[0], ret[0]);
  matrix_sub(3, 2, x[0], _x[0], _ret[0]);
  /* |1 2|   |1 2 3|    | 9 12 15|
     |3 4| * |4 5 6|  = |19 26 33|
     |5 6|              |29 40 51|*/
  string name = "toyota";
  if(name == "toyota"){
    cout << "yes, it's a toyota" << endl;
  }
  for(int i = 0; i < 3; ++i){
    for(int j = 0; j < 2; ++j){
      cout << _ret[i][j] << " ";
    }
    cout << endl;
  }

  test_util t;
  for(int i = 0; i < 12; ++i){
    cout << (int)t.arr[i] << endl;
  }
  return 0;
}