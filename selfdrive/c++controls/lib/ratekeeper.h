#ifndef RATEKEEPER_H_
#define RATEKEEPER_H_
#include <unistd.h>

class RateKeeper
{
public:
  RateKeeper(double rate, double print_delay_threshold = 0.0);
  ~RateKeeper();
  int getFrame();
  int getremaining();
  bool keep_time();
  bool monitor_time();
private:
  double _interval;
  double _next_frame_time;
  double _print_delay_threshold;
  int _frame;
  double _remaining;
};

#endif // RATEKEEPER_H_
