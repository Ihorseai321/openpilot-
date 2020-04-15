#include <cassert>

#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include "cereal/gen/cpp/log.capnp.h"
#include "cereal/gen/cpp/car.capnp.h"
#include "common/timing.h"
#include "messaging.hpp"
#include <iostream>
#include <map>
#include <vector>
#include <string.h>
#include <unistd.h>
#include "lib/interface.h"
using namespace std;

#define ENABLE 1
#define PRE_ENABLE 2
#define NO_ENTRY 3
#define WARNING 4
#define USER_DISABLE 5
#define SOFT_DISABLE 6
#define IMMEDIATE_DISABLE 7
#define PERMANENT 8
#define RECV_SIZE (0x1000)

void send(PubSocket *controls_state_sock, PubSocket *car_control_sock)
{
    capnp::MallocMessageBuilder cc_msg;
    cereal::Event::Builder cc_event = cc_msg.initRoot<cereal::Event>();
    cc_event.setLogMonoTime(nanos_since_boot());
    auto cc_send = cc_event.initCarControl();

    cc_send.setEnabled(true);
    cereal::CarControl::Actuators::Builder actuators = cc_send.initActuators();
    actuators.setGas(5.1);
                
    auto words = capnp::messageToFlatArray(cc_msg);
    auto bytes = words.asBytes();
    car_control_sock->send((char*)bytes.begin(), bytes.size());

    capnp::MallocMessageBuilder ctls_msg;
    cereal::Event::Builder ctls_event = ctls_msg.initRoot<cereal::Event>();
    ctls_event.setLogMonoTime(nanos_since_boot());
    ctls_event.setValid(true);
    auto ctls_send = ctls_event.initControlsState();
    cout << "-----------------------ctls_msg------------------" << endl;

    cereal::ControlsState::LateralControlState::Builder lat_ctls = ctls_send.initLateralControlState();
    cereal::ControlsState::LateralPIDState::Builder pid_state = lat_ctls.initPidState();
    pid_state.setActive(true);
    
    auto ctls_words = capnp::messageToFlatArray(ctls_msg);
    auto ctls_bytes = ctls_words.asBytes();

    controls_state_sock->send((char*)ctls_bytes.begin(), ctls_bytes.size());
}

std::vector<std::string> drain_sock_raw(SubSocket *sock, bool wait_for_one)
{
  std::vector<std::string> ret;
  

  while(true){
    
    std::string dat;
    
    if(wait_for_one && ret.size() == 0){
      cout << "drain_sock_raw->receive-if-before" << endl;
      Message *canmsg = sock->receive();
      cout << "drain_sock_raw->receive-if->" << endl;
      if(canmsg != NULL){
        dat.assign(canmsg->getData(), canmsg->getSize());
        cout << "drain_sock_raw->if-if" << endl;
      }
      delete canmsg;
      cout << "drain_sock_raw->delete-1" << endl;
    }
    else{
      cout << "drain_sock_raw->receive-else-before" << endl;
      Message *canmsg = sock->receive(true);
      cout << "drain_sock_raw->receive-else->" << endl;
      if(canmsg != NULL){
        dat.assign(canmsg->getData(), canmsg->getSize());
        cout << "drain_sock_raw->else-if" << endl;
      }   
      delete canmsg;  
      cout << "drain_sock_raw->delete-2" << endl; 
    }

    if(dat.empty()){
      cout << "drain_sock_raw->empty" << endl;
      break;
    }
    
    cout << dat << endl;
    cout << "--------> before push_back: " << ret.size() << endl;
    ret.push_back(dat);
    cout << "--------> after push_back: " << ret.size() << endl;
    
  }
  cout << "drain_sock_raw->return" << endl;
  return ret;
}

int main(int argc, char const *argv[])
{
    CarInterface CI(false);

    Context * c = Context::create();
    
    SubSocket *can_sock = SubSocket::create(c, "can");

    assert(can_sock != NULL);

    Poller * poller = Poller::create({can_sock});
    int running_count = 0;

    while(true){
        std::vector<std::string> can_strs = drain_sock_raw(can_sock, true);
        CARSTATE CS = CI.update(can_strs);
        sleep(2);
    }

    // while (true){
    //     for (auto s : poller->poll(0)){
    //         Message * msg = s->receive();
    //         auto amsg = kj::heapArray<capnp::word>((msg->getSize() / sizeof(capnp::word)) + 1);
    //         memcpy(amsg.begin(), msg->getData(), msg->getSize());
    //         capnp::FlatArrayMessageReader capnp_msg(amsg);
    //         cereal::Event::Reader event = capnp_msg.getRoot<cereal::Event>();
    //         auto cans = event.getCan();

    //         for(auto can : cans){
    //             auto addr = can.getAddress();
    //             auto dat = can.getDat();
    //             cout << addr << endl;
    //             cout << sizeof(dat) << ", " << dat.size() << endl;
    //         }
    //         delete msg;
    //     }

        
    //     ++running_count;
    // }

    return 0;
}
