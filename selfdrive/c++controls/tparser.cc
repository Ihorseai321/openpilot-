#include "canparser.cc"

int main()
{
  int bus = 0;
  CANParser can(bus, "ford_fusion_2018_pt", message_options_v, signal_options_v);
}