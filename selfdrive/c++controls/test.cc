#include <cassert>

#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include "cereal/gen/cpp/log.capnp.h"
#include "messaging.hpp"
#include <iostream>
#include "handler.h"

using namespace std;

int main(int argc, char const *argv[])
{
    Handler handler;
    Context * c = Context::create();
    SubSocket *car_state_sock = SubSocket::create(c, "carState");
    SubSocket *controls_state_sock = SubSocket::create(c, "controlsState");
    SubSocket *radar_state_sock = SubSocket::create(c, "radarState");
    SubSocket *model_sock = SubSocket::create(c, "model");
    SubSocket *live_parameters_sock = SubSocket::create(c, "liveParameters");
    
    assert(car_state_sock != NULL);
    assert(controls_state_sock != NULL);
    assert(radar_state_sock != NULL);
    assert(model_sock != NULL);
    assert(live_parameters_sock != NULL);

    assert(controls_state_sock != NULL);

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
            // cereal::ControlsState::LongControlState::OFF
            // cout << "running_count: " << running_count << ", " << handler.l_prob << endl;

            delete msg;
        }
        ++running_count;
    }

    return 0;
}
