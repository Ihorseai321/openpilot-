#include <cassert>

#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include "cereal/gen/cpp/log.capnp.h"
#include "cereal/gen/cpp/car.capnp.h"
#include "common/timing.h"
#include "messaging.hpp"
#include <iostream>
#include "lib/handler.h"
#include <map>
#include <vector>
#include <string.h>
using namespace std;

#define ENABLE 1
#define PRE_ENABLE 2
#define NO_ENTRY 3
#define WARNING 4
#define USER_DISABLE 5
#define SOFT_DISABLE 6
#define IMMEDIATE_DISABLE 7
#define PERMANENT 8
map<string, vector<int>> events;

// capnp::List<cereal::CarEvent>::Builder setEvents()
// {
//   capnp::MallocMessageBuilder events_msg;
//   cereal::Event::Builder event = events_msg.initRoot<cereal::Event>();
//   event.setLogMonoTime(nanos_since_boot());
//   auto events_send = event.initCarEvents();

//   capnp::List<cereal::CarEvent>::Builder list;
//   cereal::CarEvent::Builder car_event(nullptr);
//   car_event.setName(cereal::CarEvent::EventName::PEDAL_PRESSED); 
//   car_event.setEnable(true);
//   list.set(0, car_event);
//   return list;
// }
std::string drain_sock_raw(SubSocket *sock)
{
  std::string ret;
  Message *msg;
  while(true){
    if(ret.empty()){
        msg = sock->receive();
      // msg->init(sock->receive()->getData(),sock->receive()->getSize());
        if(msg->getSize() > 0){
            cout << "sock-----------------> " << msg->getSize() << endl;
            for(int i = 0; i < msg->getSize(); ++i){
                ret += msg->getData()[i];
                // cout << " " << int(sock->receive()->getData()[i]);
            }
            // cout << endl;
        }
    }
    else{
      break;
    }
  }

  return ret;
}

int main(int argc, char const *argv[])
{
    string name = "preLaneChangeLeft";
    vector<int> type;
    type.push_back(WARNING);
    events.insert(std::map<string, vector<int>>::value_type("preLaneChangeLeft", type));
    Handler handler;
    Context * c = Context::create();
    SubSocket *car_state_sock = SubSocket::create(c, "carState");
    SubSocket *controls_state_sock = SubSocket::create(c, "controlsState");
    SubSocket *radar_state_sock = SubSocket::create(c, "radarState");
    SubSocket *model_sock = SubSocket::create(c, "model");
    SubSocket *live_parameters_sock = SubSocket::create(c, "liveParameters");
    SubSocket *can_sock = SubSocket::create(c, "can");
    PubSocket *car_control_sock = PubSocket::create(c, "carControl");

    
    assert(car_state_sock != NULL);
    assert(controls_state_sock != NULL);
    assert(radar_state_sock != NULL);
    assert(model_sock != NULL);
    assert(live_parameters_sock != NULL);

    assert(controls_state_sock != NULL);

    assert(can_sock != NULL);
    assert(car_control_sock != NULL);

    Poller * poller = Poller::create({car_state_sock, controls_state_sock, radar_state_sock, model_sock, live_parameters_sock});
    
    int running_count = 0;



    while (true){
        for (auto s : poller->poll(100)){
            Message * msg = s->receive();
            auto amsg = kj::heapArray<capnp::word>((msg->getSize() / sizeof(capnp::word)) + 1);
            memcpy(amsg.begin(), msg->getData(), msg->getSize());
            capnp::FlatArrayMessageReader capnp_msg(amsg);
            cereal::Event::Reader event = capnp_msg.getRoot<cereal::Event>();

            handler.handle_log(event);

            for (int i = 0; i < 50; ++i)
            {
                cout << handler.l_points[i] << ", ";
            }
            cout << endl;

            std::string can_strs = drain_sock_raw(can_sock);
            cout << "------------>can_strs: " << can_strs.size() << endl;
            cereal::CarEvent::EventName eventname;
            cereal::ControlsState::LongControlState LongCtrlState = cereal::ControlsState::LongControlState::OFF;
            if(LongCtrlState == (cereal::ControlsState::LongControlState)0){ // cereal::ControlsState::LongControlState::OFF){
                // cereal::CarEvent::Builder ce(nullptr);
                // ce = setEvent();
                // bool enable = ce.getEnable();
                // cout << "car_event.enable: " << enable << endl;
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

                cout << "running_count: " << running_count << ", " << handler.l_prob << endl;
            }
            
            // event.getCan();
            
            delete msg;
        }
        ++running_count;
    }

    return 0;
}
