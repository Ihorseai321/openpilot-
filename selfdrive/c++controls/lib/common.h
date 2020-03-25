#ifndef COMMON_H_
#define COMMON_H_

#include <vector>
#include <map>
#include <unordered_map>

#include "common_dbc.h"
#include <capnp/serialize.h>
#include "cereal/gen/cpp/log.capnp.h"
#define DEBUG(...)
// #define DEBUG printf
#define INFO printf
#define MAX_BAD_COUNTER 5

// Helper functions
unsigned int honda_checksum(unsigned int address, uint64_t d, int l);
unsigned int toyota_checksum(unsigned int address, uint64_t d, int l);
void init_crc_lookup_tables();
unsigned int volkswagen_crc(unsigned int address, uint64_t d, int l);
unsigned int pedal_checksum(uint64_t d, int l);
uint64_t read_u64_be(const uint8_t* v);
uint64_t read_u64_le(const uint8_t* v);

// class CANPacker {
// private:
//   const DBC *dbc = NULL;
//   std::map<std::pair<uint32_t, std::string>, Signal> signal_lookup;
//   std::map<uint32_t, Msg> message_lookup;

// public:
//   CANPacker(const std::string& dbc_name);
//   uint64_t pack(uint32_t address, const std::vector<SignalPackValue> &signals, int counter);
// };
#endif //COMMON_H_