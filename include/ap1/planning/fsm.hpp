#ifndef AP1_PLANNING_FSM_HPP
#define AP1_PLANNING_FSM_HPP

#include <vector>
#include <string>

namespace ap1::planning::fsm {
    enum class VehicleState {
        Driving,
        Stopping,
        Stopped,
        DrivingThrough
    };

    enum class Event {
        SignDetected, // if a sign **THAT APPLIES** has been detected. Not just any sign.
        HasStopped, // v = 0
        StopTimeElapsed, // timer
        DriveThruDistanceCovered
    };

    struct Transition {
        VehicleState from;
        Event event;
        VehicleState to;
    };

    struct FSM {
        VehicleState current_state;
        std::vector<Transition> transitions;
    };

    /**
     * @brief Loads a transition table from a yaml file.
     * 
     * @param path 
     * @return std::vector<Transition> 
     */
    std::vector<Transition> load_transitions_from_file(std::string path);

    /**
     * @brief Transitions to the next state based on a transition table and current state.
     * 
     * If no transition is found for the current state and event, the current state is returned back.
     */
    VehicleState next_state(VehicleState current, std::vector<Event> events, const std::vector<Transition> &transitions);
} // ap1::planning::fsm
#endif // AP1_PLANNING_FSM_HPP