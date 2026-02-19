#include "ap1/planning/fsm.hpp"

#include <string>
#include <vector>
#include <stdexcept>
#include <filesystem>
#include <unordered_map>

#include <yaml-cpp/yaml.h>

using namespace ap1::planning::fsm;

// Helper funcs
VehicleState state_from_string(const std::string &s);
Event event_from_string(const std::string &s);

// Beef and potatoes
VehicleState next_state(VehicleState current, Event e, const std::vector<Transition> &transitions) {
    for (const auto& transition : transitions) {
        if (transition.from == current && transition.event == e) {
            return transition.to;
        }
    }

    // otherwise just stay in the current state
    return current;
}

std::vector<Transition> load_transitions_from_file(std::string path) {
    // 1. Check extension
    std::filesystem::path fs_path(path);
    if (fs_path.extension() != ".yaml") {
        throw std::runtime_error("Invalid file extension. Expected .yaml");
    }

    // Load that bomboclaat file
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception &e) {
        throw std::runtime_error(
            std::string("Failed to load YAML file: ") + e.what()
        );
    }

    // Make sure we have a root
    if (!root["transitions"]) {
        throw std::runtime_error("Missing 'transitions' in config.");
    }

    // Build transitions vector
    std::vector<Transition> transitions;

    for (const auto& node : root["transitions"]) {
        if (!node.IsMap()) {
            throw std::runtime_error("Each transition must be a map");
        }

        if (!node["from"] || !node["event"] || !node["to"]) {
            throw std::runtime_error("Transntion is missing fields. Must have 'from', 'event', to'");
        }

        std::string from_str = node["from"].as<std::string>();
        std::string event_str = node["event"].as<std::string>();
        std::string to_str = node["to"].as<std::string>();

        // validate against enum options
        VehicleState from = state_from_string(from_str);
        Event event = event_from_string(event_str);
        VehicleState to = state_from_string(to_str);
    
        transitions.push_back({from, event, to});
    }

    if (transitions.empty()) {
        throw std::runtime_error("No transitions defined in file");
    }

    return transitions;
}

// Helpers
VehicleState state_from_string(const std::string &s) {
    static const std::unordered_map<std::string, VehicleState> map{
        {"Driving", VehicleState::Driving},
        {"Stopping", VehicleState::Stopping},
        {"Stopped", VehicleState::Stopped},
        {"DrivingThrough", VehicleState::DrivingThrough}
    };

    auto it = map.find(s);
    if (it == map.end()) {
        throw std::runtime_error("Invalid state: " + s);
    }

    return it->second;
}

Event event_from_string(const std::string& s) {
    static const std::unordered_map<std::string, Event> map{
        {"SignDetected", Event::SignDetected},
        {"HasStopped", Event::HasStopped},
        {"StopTimeElapsed", Event::StopTimeElapsed},
        {"DriveThruDistanceCovered", Event::DriveThruDistanceCovered}
    };

    auto it = map.find(s);
    if (it == map.end()) {
        throw std::runtime_error("Invalid event: " + s);
    }

    return it->second;
}