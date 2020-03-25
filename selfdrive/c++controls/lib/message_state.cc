#include "message_state.h"
#include "common.h"

bool MessageState::parse(uint64_t sec, uint16_t ts_, uint8_t * dat) {
  uint64_t dat_le = read_u64_le(dat);
  uint64_t dat_be = read_u64_be(dat);

  for (int i=0; i < parse_sigs.size(); i++) {
    auto& sig = parse_sigs[i];
    int64_t tmp;

    if (sig.is_little_endian){
      tmp = (dat_le >> sig.b1) & ((1ULL << sig.b2)-1);
    } else {
      tmp = (dat_be >> sig.bo) & ((1ULL << sig.b2)-1);
    }

    if (sig.is_signed) {
      tmp -= (tmp >> (sig.b2-1)) ? (1ULL << sig.b2) : 0; //signed
    }

    DEBUG("parse 0x%X %s -> %lld\n", address, sig.name, tmp);

    if (sig.type == SignalType::HONDA_CHECKSUM) {
      if (honda_checksum(address, dat_be, size) != tmp) {
        INFO("0x%X CHECKSUM FAIL\n", address);
        return false;
      }
    } else if (sig.type == SignalType::HONDA_COUNTER) {
      if (!update_counter_generic(tmp, sig.b2)) {
        return false;
      }
    } else if (sig.type == SignalType::TOYOTA_CHECKSUM) {
      if (toyota_checksum(address, dat_be, size) != tmp) {
        INFO("0x%X CHECKSUM FAIL\n", address);
        return false;
      }
    } else if (sig.type == SignalType::VOLKSWAGEN_CHECKSUM) {
      if (volkswagen_crc(address, dat_le, size) != tmp) {
        INFO("0x%X CRC FAIL\n", address);
        return false;
      }
    } else if (sig.type == SignalType::VOLKSWAGEN_COUNTER) {
        if (!update_counter_generic(tmp, sig.b2)) {
        return false;
      }
    } else if (sig.type == SignalType::PEDAL_CHECKSUM) {
      if (pedal_checksum(dat_be, size) != tmp) {
        INFO("0x%X PEDAL CHECKSUM FAIL\n", address);
        return false;
      }
    } else if (sig.type == SignalType::PEDAL_COUNTER) {
      if (!update_counter_generic(tmp, sig.b2)) {
        return false;
      }
    }

    vals[i] = tmp * sig.factor + sig.offset;
  }
  ts = ts_;
  seen = sec;

  return true;
}


bool MessageState::update_counter_generic(int64_t v, int cnt_size) {
  uint8_t old_counter = counter;
  counter = v;
  if (((old_counter+1) & ((1 << cnt_size) -1)) != v) {
    counter_fail += 1;
    if (counter_fail > 1) {
      INFO("0x%X COUNTER FAIL %d -- %d vs %d\n", address, counter_fail, old_counter, (int)v);
    }
    if (counter_fail >= MAX_BAD_COUNTER) {
      return false;
    }
  } else if (counter_fail > 0) {
    counter_fail--;
  }
  return true;
}
