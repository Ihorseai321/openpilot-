#include <cassert>

#include <capnp/message.h>
#include <capnp/serialize-packed.h>
#include "cereal/gen/cpp/log.capnp.h"
#include "messaging.hpp"
#include "common/timing.h"

#define ARRAYSIZE(x) (sizeof(x)/sizeof(x[0]))

int main(int argc, char const *argv[])
{
    Context * c = Context::create();
    PubSocket *pathplan_sock = PubSocket::create(c, "pathPlan");
    assert(pathplan_sock != NULL);

    float laneWidth = 3.7;

    float dPoly[4] = {1.0, 1.0, 1.0, 1.0};
    float cPoly[4] = {2.0, 2.0, 2.0, 2.0};
    float cProb = 1.0;
    float lPoly[4] = {3.0, 3.0, 3.0, 3.0};
    float lProb = 1.0;
    float rPoly[4] = {4.0, 4.0, 4.0, 4.0};
    float rProb = 1.0;

    float angleSteers = 80.0; // deg
    float rateSteers = 56.0; // deg/s
    bool mpcSolutionValid = true;
    bool paramsValid = true;
    bool modelValidDEPRECATED = true;
    float angleOffset = 1.0;
    bool sensorValid = true;
    bool commIssue = true;
    bool posenetValid = true;
    cereal::PathPlan::Desire desire = cereal::PathPlan::Desire::NONE;
    cereal::PathPlan::LaneChangeState laneChangeState = cereal::PathPlan::LaneChangeState::OFF;
    cereal::PathPlan::LaneChangeDirection laneChangeDirection = cereal::PathPlan::LaneChangeDirection::NONE;
    
    while (true){
        capnp::MallocMessageBuilder pp_msg;
        cereal::Event::Builder event = pp_msg.initRoot<cereal::Event>();
        event.setLogMonoTime(nanos_since_boot());
        auto plan_send = event.initPathPlan();

        kj::ArrayPtr<const float> d_Poly(&dPoly[0], ARRAYSIZE(dPoly));
        kj::ArrayPtr<const float> l_Poly(&lPoly[0], ARRAYSIZE(lPoly));
        kj::ArrayPtr<const float> r_Poly(&rPoly[0], ARRAYSIZE(rPoly));

        // capnp::List<float>::Builder d_Poly = plan_send.initDPoly(4);
        // capnp::List<float>::Builder l_Poly = plan_send.initLPoly(4);
        // capnp::List<float>::Builder r_Poly = plan_send.initRPoly(4);

        // d_Poly.set(0, 1);
        // d_Poly.set(1, 1);
        // d_Poly.set(2, 1);
        // d_Poly.set(3, 1);

        // for (int i = 0; i < 4; ++i){
        //     d_Poly[i] = dPoly[i];
        //     l_Poly[i] = lPoly[i];
        //     r_Poly[i] = rPoly[i];
        // }        

        plan_send.setLaneWidth(laneWidth);
        plan_send.setDPoly(d_Poly);
        plan_send.setLPoly(l_Poly);
        plan_send.setRPoly(r_Poly);
        plan_send.setLProb(lProb);
        plan_send.setRProb(rProb);
    
        plan_send.setAngleSteers(angleSteers);
        plan_send.setRateSteers(rateSteers);
        plan_send.setAngleOffset(angleOffset);
        plan_send.setMpcSolutionValid(mpcSolutionValid);
        plan_send.setParamsValid(paramsValid);
        plan_send.setSensorValid(sensorValid);
        plan_send.setPosenetValid(posenetValid);

        plan_send.setDesire(desire);
        plan_send.setLaneChangeState(laneChangeState);
        plan_send.setLaneChangeDirection(laneChangeDirection);

        auto words = capnp::messageToFlatArray(pp_msg);
        auto bytes = words.asBytes();
        pathplan_sock->send((char*)bytes.begin(), bytes.size());

    }

    return 0;
}
