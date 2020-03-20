#ifndef CANPARSER_H_
#define CANPARSER_H_

#include <unordered_map>
#include <unordered_set>
#include <string>
#include <any>
#include <iostream>
#include <vector>
#include "common.h"

#define CAN_INVALID_CNT 5

typedef struct{
  std::string sig_name;
  std::string sig_address;
  double sig_default;
}SIGNAL;

class Parser
{
public:
  Parser(std::string dbc_name, SINGNAL signals[], int bus);
  ~Parser();
  std::unordered_set<uint32_t> update_vl();
  std::unordered_set<uint32_t> update_string(std::string dat, bool sendcan);
  std::unordered_set<uint32_t> update_strings(std::vector<std::string> strings, bool sendcan)

  string dbc_name;
  std::unordered_map<std::any, std::map<std::string, double>> vl;
  std::unordered_map<std::any, std::map<std::string, uint16_t>> ts;
  bool can_valid;
  int can_invalid_cnt;
private:
  DBC *dbc;
  CANParser *can;
  std::map<std::string, unsigned int> msg_name_to_address;
  std::map<unsigned int, std::string> address_to_msg_name;
  std::vector<SignalValue> can_values;
  bool test_mode_enabled;
};
#endif