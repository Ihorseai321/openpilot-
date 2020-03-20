#include "ratekeeper.h"
#include "utils.h"

RateKeeper::RateKeeper(float rate, float print_delay_threshold)
{
  _interval = 1.0 / rate;
  _next_frame_time = sec_since_boot() + _interval;
  _print_delay_threshold = print_delay_threshold;
  _frame = 0;
  _remaining = 0;
}

RateKeeper::~RateKeeper()
{}

int RateKeeper::getFrame()
{
  return _frame;
}

int remaining()
{
  return _remaining;
}

bool keep_time()
{
  bool lagged = monitor_time();
  if(_remaining > 0){
    usleep(_remaining * 1000000);
  }
  return lagged;
}

bool monitor_time()
{
  bool lagged = false;
  double remaining = _next_frame_time - sec_since_boot();
  _next_frame_time += _interval;

  if(_print_delay_threshold && remaining < -_print_delay_threshold){
    lagged = true;
  }
  _frame += 1;
  _remaining = remaining;

  return lagged;
}