#ifndef INTERFACE_H_
#define INTERFACE_H_

class CarInterface
{

public:
  CarInterface();
  ~CarInterface();
  
  int frame;
  bool gas_pressed_prev;
  bool brake_pressed_prev;
  bool cruise_enabled_prev;
private:
  CarParams CP;
  VehicleModel VM;
  CarState CS;
};
#endif // INTERFACE_H_