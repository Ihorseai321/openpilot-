#ifndef MESSAGE_STATE_H_
#define MESSAGE_STATE_H_
#include <cstddef>
#include <cstdint>
#include <vector>
#include "common.h"

class MessageState {
public:
  uint32_t address;
  unsigned int size;

  std::vector<Signal> parse_sigs;
  std::vector<double> vals;

  uint16_t ts;
  uint64_t seen;
  uint64_t check_threshold;

  uint8_t counter;
  uint8_t counter_fail;

  bool parse(uint64_t sec, uint16_t ts_, uint8_t * dat);
  bool update_counter_generic(int64_t v, int cnt_size);
};

#endif //MESSAGE_STATE_H_