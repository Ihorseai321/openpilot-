#ifndef CANDEFINE_H_
#define CANDEFINE_H_

class CANDefine
{
public:
  CANDefine();
  ~CANDefine();
  const DBC *dbc
  dict dv;
  std::string dbc_name;
  
};    

  def __init__(self, dbc_name):
    self.dbc_name = dbc_name
    self.dbc = dbc_lookup(dbc_name)

    num_vals = self.dbc[0].num_vals

    address_to_msg_name = {}

    num_msgs = self.dbc[0].num_msgs
    for i in range(num_msgs):
      msg = self.dbc[0].msgs[i]
      name = msg.name.decode('utf8')
      address = msg.address
      address_to_msg_name[address] = name

    dv = defaultdict(dict)

    for i in range(num_vals):
      val = self.dbc[0].vals[i]

      sgname = val.name.decode('utf8')
      address = val.address
      def_val = val.def_val.decode('utf8')

      //separate definition/value pairs
      def_val = def_val.split()
      values = [int(v) for v in def_val[::2]]
      defs = def_val[1::2]

      if address not in dv:
        dv[address] = {}
        msgname = address_to_msg_name[address]
        dv[msgname] = {}

      // two ways to lookup: address or msg name
      dv[address][sgname] = dict(zip(values, defs))
      dv[msgname][sgname] = dv[address][sgname]

      self.dv = dict(dv)
#endif // CANDEFINE_H_