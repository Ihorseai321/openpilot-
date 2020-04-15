#include <cassert>
#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include "cereal/gen/cpp/log.capnp.h"
#include "messaging.hpp"
#include "lib/pathplanner.h"
#include "lib/planner.h"
#include "lib/handler.h"
#include "lib/vehicle_model.h"

#include <iostream>
using namespace std;

int main(int argc, char const *argv[])
{
    // set_realtime_priority(2);
    PathPlanner PP;
    Planner PL; 
    VehicleModel VM;
    Handler handler;

    Context * c = Context::create();
    SubSocket *car_state_sock = SubSocket::create(c, "carState");
    SubSocket *controls_state_sock = SubSocket::create(c, "controlsState");
    SubSocket *radar_state_sock = SubSocket::create(c, "radarState");
    SubSocket *model_sock = SubSocket::create(c, "model");
    SubSocket *live_parameters_sock = SubSocket::create(c, "liveParameters");
    PubSocket *plan_sock = PubSocket::create(c, "plan");
    PubSocket *livelongitudinalmpc_sock = PubSocket::create(c, "liveLongitudinalMpc");
    PubSocket *pathplan_sock = PubSocket::create(c, "pathPlan");
    PubSocket *livempc_sock = PubSocket::create(c, "liveMpc");
    
    assert(car_state_sock != NULL);
    assert(controls_state_sock != NULL);
    assert(radar_state_sock != NULL);
    assert(model_sock != NULL);
    assert(live_parameters_sock != NULL);
    assert(plan_sock != NULL);
    assert(livelongitudinalmpc_sock != NULL);
    assert(pathplan_sock != NULL);
    assert(livempc_sock != NULL);

    Poller * poller = Poller::create({car_state_sock, controls_state_sock, radar_state_sock, model_sock, live_parameters_sock});

    // sm['liveParameters'].valid = True
    // sm['liveParameters'].sensorValid = True
    // sm['liveParameters'].steerRatio = CP.steerRatio
    // sm['liveParameters'].stiffnessFactor = 1.0
    
    while (true){
      // cout << "plannerd running!!!" << endl;
      for (auto s : poller->poll(100)){
        Message * msg = s->receive();
        auto amsg = kj::heapArray<capnp::word>((msg->getSize() / sizeof(capnp::word)) + 1);
        memcpy(amsg.begin(), msg->getData(), msg->getSize());
        capnp::FlatArrayMessageReader capnp_msg(amsg);
        cereal::Event::Reader event = capnp_msg.getRoot<cereal::Event>();

        handler.handle_log(event);

        auto which = event.which();
        if (which == cereal::Event::MODEL){
          PP.update(handler, pathplan_sock, livempc_sock, VM);
        } else if (which == cereal::Event::RADAR_STATE){
          PL.update(handler, plan_sock, livelongitudinalmpc_sock);
        }
        delete msg;
      }
    }
    
    delete car_state_sock;
    delete controls_state_sock;
    delete radar_state_sock;
    delete model_sock;
    delete live_parameters_sock;
    delete plan_sock;
    delete livelongitudinalmpc_sock;
    delete pathplan_sock;
    delete livempc_sock;
    delete poller;
    delete c;

    return 0;
}