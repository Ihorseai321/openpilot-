#include <vector>

#include "common_dbc.h"

namespace {

std::vector<DBC*>& get_dbcs() {
  static std::vector<DBC*> vec;
  return vec;
}

}

DBC* dbc_lookup(const std::string& dbc_name) {
  for (auto& dbci : get_dbcs()) {
    if (dbc_name == dbci->name) {
      return dbci;
    }
  }
  return NULL;
}

void dbc_register(DBC* dbc) {
  get_dbcs().push_back(dbc);
}

extern "C" {
  DBC* dbc_lookup(const char* dbc_name) {
    return dbc_lookup(std::string(dbc_name));
  }
}
