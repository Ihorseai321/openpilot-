#include "parser.h"

CANParser::CANParser(int abus, const std::string& dbc_name,
          const std::vector<MessageParseOptions> &options,
          const std::vector<SignalParseOptions> &sigoptions)
  : bus(abus) {

  dbc = dbc_lookup(dbc_name);
  assert(dbc);
  init_crc_lookup_tables();

  for (const auto& op : options) {
    MessageState state = {
      .address = op.address,
      // .check_frequency = op.check_frequency,
    };

    // msg is not valid if a message isn't received for 10 consecutive steps
    if (op.check_frequency > 0) {
      state.check_threshold = (1000000000ULL / op.check_frequency) * 10;
    }


    const Msg* msg = NULL;
    for (int i=0; i<dbc->num_msgs; i++) {
      if (dbc->msgs[i].address == op.address) {
        msg = &dbc->msgs[i];
        break;
      }
    }
    if (!msg) {
      fprintf(stderr, "CANParser: could not find message 0x%X in DBC %s\n", op.address, dbc_name.c_str());
      assert(false);
    }

    state.size = msg->size;

    // track checksums and counters for this message
    for (int i=0; i<msg->num_sigs; i++) {
      const Signal *sig = &msg->sigs[i];
      if (sig->type != SignalType::DEFAULT) {
        state.parse_sigs.push_back(*sig);
        state.vals.push_back(0);
      }
    }

    // track requested signals for this message
    for (const auto& sigop : sigoptions) {
      if (sigop.address != op.address) continue;

      for (int i=0; i<msg->num_sigs; i++) {
        const Signal *sig = &msg->sigs[i];
        if (strcmp(sig->name, sigop.name) == 0
            && sig->type == SignalType::DEFAULT) {
          state.parse_sigs.push_back(*sig);
          state.vals.push_back(sigop.default_value);
          break;
        }
      }

    }

    message_states[state.address] = state;
  }
}

void CANParser::UpdateCans(uint64_t sec, const capnp::List<cereal::CanData>::Reader& cans) {
    int msg_count = cans.size();
    uint64_t p;

    DEBUG("got %d messages\n", msg_count);

    // parse the messages
    for (int i = 0; i < msg_count; i++) {
      auto cmsg = cans[i];
      if (cmsg.getSrc() != bus) {
        // DEBUG("skip %d: wrong bus\n", cmsg.getAddress());
        continue;
      }
      auto state_it = message_states.find(cmsg.getAddress());
      if (state_it == message_states.end()) {
        // DEBUG("skip %d: not specified\n", cmsg.getAddress());
        continue;
      }

      if (cmsg.getDat().size() > 8) continue; //shouldnt ever happen
      uint8_t dat[8] = {0};
      memcpy(dat, cmsg.getDat().begin(), cmsg.getDat().size());

      state_it->second.parse(sec, cmsg.getBusTime(), dat);
    }
}

void CANParser::UpdateValid(uint64_t sec) {
  can_valid = true;
  for (const auto& kv : message_states) {
    const auto& state = kv.second;
    if (state.check_threshold > 0 && (sec - state.seen) > state.check_threshold) {
      if (state.seen > 0) {
        DEBUG("0x%X TIMEOUT\n", state.address);
      }
      can_valid = false;
    }
  }
}

void CANParser::update_string(std::string data, bool sendcan) {
  // format for board, make copy due to alignment issues, will be freed on out of scope
  auto amsg = kj::heapArray<capnp::word>((data.length() / sizeof(capnp::word)) + 1);
  memcpy(amsg.begin(), data.data(), data.length());

  // extract the messages
  capnp::FlatArrayMessageReader cmsg(amsg);
  cereal::Event::Reader event = cmsg.getRoot<cereal::Event>();

  last_sec = event.getLogMonoTime();

  auto cans = sendcan? event.getSendcan() : event.getCan();
  UpdateCans(last_sec, cans);

  UpdateValid(last_sec);
}


std::vector<SignalValue> CANParser::query_latest() {
  std::vector<SignalValue> ret;

  for (const auto& kv : message_states) {
    const auto& state = kv.second;
    if (last_sec != 0 && state.seen != last_sec) continue;

    for (int i=0; i<state.parse_sigs.size(); i++) {
      const Signal &sig = state.parse_sigs[i];
      ret.push_back((SignalValue){
        .address = state.address,
        .ts = state.ts,
        .name = sig.name,
        .value = state.vals[i],
      });
    }
  }

  return ret;
}
