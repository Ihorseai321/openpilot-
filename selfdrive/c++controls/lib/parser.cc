#include "parser.h"

Parser::Parser(std::string dbc_name, SIGNAL signals[], int bus)
{
  can_valid = true;
  this->dbc_name = dbc_name;
  dbc = dbc_lookup(dbc_name);
  can_invalid_cnt = CAN_INVALID_CNT;
  size_t num_msgs = dbc->num_msgs;

  for(int i = 0; i < num_msgs; ++i){
    struct Msg msg = dbc->msgs[i];
    
    std::string name = msg.name;
    msg_name_to_address.insert(std::map<std::string, unsigned int>::value_type(name, msg.address));
    address_to_msg_name.insert(std::map<unsigned int, std::string>::value_type(msg.address, name));
  }
  
  std::vector<struct SignalParseOptions> signal_options_v;
  
  
  std::vector<struct MessageParseOptions> message_options_v;
  struct MessageParseOptions mpo;
  for(int j = 0; j < 25; ++j){
    struct SignalParseOptions spo = {msg_name_to_address[signals[j].sig_address], signals[j].sig_name.c_str(), signals[j].sig_default};
    // spo.address = msg_name_to_address[signals[j].sig_address];
    // spo.name = signals[j].sig_name.c_str();
    // spo.default_value = signals[j].sig_default;
    signal_options_v.push_back(spo);

    mpo.address = msg_name_to_address[signals[j].sig_address];
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

std::unordered_set<unsigned int> Parser::update_vl()
{
  std::string sig_name;
  std::unordered_set<unsigned int> updated_val;

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

    std::pair<std::string, std::string> cvpair;
    cvpair = make_pair(name, cv_name);
    
    vl.insert(std::map<std::pair<std::string, std::string>, double>::value_type(cvpair, cv.value));
    ts.insert(std::map<std::pair<std::string, std::string>, unsigned int>::value_type(cvpair, cv.ts));

    updated_val.insert(cv.address);
  }

  return updated_val;
}

std::unordered_set<unsigned int> Parser::update_string(std::string dat, bool sendcan)
{
  can->update_string(dat, sendcan);
  return update_vl();
}

// std::unordered_set<unsigned int> Parser::update_strings(std::vector<std::string> strings, bool sendcan)
std::unordered_set<unsigned int> Parser::update_strings(std::vector<std::string> strings, bool sendcan)
{
  std::unordered_set<unsigned int> updated_vals;
  std::unordered_set<unsigned int> updated_val;

  // updated_val = update_string(s, sendcan);
  // updated_vals.insert(updated_val.begin(), updated_val.end());
  for(auto s : strings){
    updated_val = update_string(s, sendcan);
    updated_vals.insert(updated_val.begin(), updated_val.end());
  }

  return updated_vals;
}