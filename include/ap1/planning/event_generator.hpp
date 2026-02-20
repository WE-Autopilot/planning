/**
 * Event generator is responsible for determining events that occur in the world through mapping frames.
 * These are then consumed
 */

#ifndef AP1_PLANNING_EVENT_GEN_HPP
#define AP1_PLANNING_EVENT_GEN_HPP

#include <optional>

#include "ap1/planning/fsm.hpp"

namespace ap1::planning {
class EventGenerator {
public:
    EventGenerator();

    std::optional<ap1::planning::fsm::Event> update();    
};
}

#endif // AP1_PLANNING_EVENT_GEN_HPP