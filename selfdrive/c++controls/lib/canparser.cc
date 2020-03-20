#include "canparser.h"

Parser::Parser(std::string dbc_name, SINGNAL signals[], int bus)
{
  can_valid = true;
  this->dbc_name = dbc_name;
  dbc = dbc_lookup(dbc_name);
  can_invalid_cnt = CAN_INVALID_CNT;
  size_t num_msgs = dbc.num_msgs;

  for(int i = 0; i < num_msgs; ++i){
    struct Msg msg = dbc.msgs[i];
    std::string name = msg.name;

    msg_name_to_address[name] = msg.address;
    address_to_msg_name[msg.address] = name;
    vl[msg.address] = {};
    vl[name] = {};
    ts[msg.address] = {};
    ts[name] = {};
  }
  
  std::vector<struct SignalParseOptions> signal_options_v;
  struct SignalParseOptions spo;
  
  vector<struct MessageParseOptions> message_options_v;
  struct MessageParseOptions mpo;
  for(int j = 0; j < sizeof(signals)/sizeof(signals[0]); ++j){
    SINGNAL s = signals[j];
    signals[j] = msg_name_to_address[signals[j].sig_address];

    spo.address = signals[j].sig_address;
    spo.name = signals[j].sig_name;
    spo.default_value = signals[j].sig_default;
    signal_options_v.push_back(spo);

    mpo.address = signals[j].sig_address;
    mpo.check_frequency = 0;
    message_options_v.push_back(mpo);
  }
  can = new CANParser(bus, dbc_name, message_options_v, signal_options_v);
  update_vl();
}

Parser::~Parser()
{
  delete can;
}

std::unordered_set<uint32_t> Parser::update_vl()
{
  std::string sig_name;
  std::unordered_set<uint32_t> updated_val;

  std::vector<SignalValue> can_values = can->query_latest();
  bool valid = can->can_valid;

  // Update invalid flag
  can_invalid_cnt += 1;
  if(valid){
    can_invalid_cnt = 0;
  }
  can_valid = can_invalid_cnt < CAN_INVALID_CNT;


  for(auto cv : can_values){
    // Cast char * directly to unicde
    std::string name = address_to_msg_name[cv.address];
    std::string cv_name = cv.name;
    vl[cv.address][cv_name] = cv.value;
    ts[cv.address][cv_name] = cv.ts;

    vl[name][cv_name] = cv.value;
    ts[name][cv_name] = cv.ts;
    updated_val.insert(cv.address);
  }

  return updated_val;
}

std::unordered_set<uint32_t> Parser::update_string(std::string dat, bool sendcan)
{
  can->update_string(dat, sendcan);
  return update_vl();
}

std::unordered_set<uint32_t> Parser::update_strings(std::vector<std::string> strings, bool sendcan)
{
  std::unordered_set<uint32_t> updated_vals;
  for(auto s : strings){
    updated_val = update_string(s, sendcan);
    updated_vals.insert(updated_val.begin(), updated_val.end());
  }

  return updated_vals;
}