#ifndef RATEKEEPER_H_
#define RATEKEEPER_H_

class RateKeeper
{
public:
  RateKeeper(double rate, double print_delay_threshold = 0.0);
  ~RateKeeper();
  int getFrame();
  int remaining();
  bool keep_time();
  bool monitor_time();
private:
  double _interval = 1. / rate;
  double _next_frame_time = sec_since_boot() + _interval;
  double _print_delay_threshold = print_delay_threshold;
  int _frame = 0;
  double at _remaining = 0;
};

#endif // RATEKEEPER_H_
