#ifndef PARSER_H_
#define PARSER_H_
#include <unordered_set>
#include <map>
#include <string>
#include <iostream>
#include <vector>
#include "canparser.h"
#include "utils.h"
#define CAN_INVALID_CNT 5


class Parser
{
public:
  Parser(std::string dbc_name, SIGNAL signals[], int bus);
  virtual ~Parser();
  std::unordered_set<unsigned int> update_vl();
  std::unordered_set<unsigned int> update_string(std::string dat, bool sendcan);
  std::unordered_set<unsigned int> update_strings(std::vector<std::string> strings, bool sendcan);
  // std::unordered_set<unsigned int> update_strings(std::vector<std::string> strings, bool sendcan);

  std::string dbc_name;
  std::map<std::pair<std::string, std::string>, double> vl;
  std::map<std::pair<std::string, std::string>, unsigned int> ts;
  bool can_valid;
  int can_invalid_cnt;
private:
  const DBC *dbc;
  CANParser *can;
  std::map<std::string, unsigned int> msg_name_to_address;
  std::map<unsigned int, std::string> address_to_msg_name;
  std::vector<SignalValue> can_values;
  bool test_mode_enabled;
};
#endif // PARSER_H_