#ifndef CANPARSER_H_
#define CANPARSER_H_
#include <cassert>
#include <cstring>

#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <algorithm>
#include "common.h"
#include "message_state.h"
#include <unordered_map>
#include <vector>

class CANParser {
private:
  const int bus;

  const DBC *dbc = NULL;
  std::unordered_map<uint32_t, MessageState> message_states;

public:
  bool can_valid = false;
  uint64_t last_sec = 0;

  CANParser(int abus, const std::string& dbc_name,
            const std::vector<MessageParseOptions> &options,
            const std::vector<SignalParseOptions> &sigoptions);
  void UpdateCans(uint64_t sec, const capnp::List<cereal::CanData>::Reader& cans);
  void UpdateValid(uint64_t sec);
  void update_string(std::string data, bool sendcan);
  std::vector<SignalValue> query_latest();
};


#endif //CANPARSER_H_