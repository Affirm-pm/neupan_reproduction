// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_common/frames.hpp"

#include <utility>

#include "finenav_util/common/log_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace finenav::common {

namespace {

// Function-local statics defined in finenav_common.so — ONE instance per
// process, shared by the framework, the visualizer and every plugin DSO that
// links the library.  This is the whole point: see the note in frames.hpp about
// why these are not inline header variables, and named_registry.hpp for the same
// discipline applied to the algorithm registries, two files over.
FrameNames& storage() {
    static FrameNames names;
    return names;
}

bool& wasSet() {
    static bool set = false;
    return set;
}

}  // namespace

const FrameNames& frames() {
    return storage();
}

const std::string& frameName(Frame frame) {
    // Total by construction: there are exactly two frames and the switch covers
    // both.  The trailing return is for the compiler, which cannot see that an
    // enum class holds nothing else.
    switch (frame) {
        case Frame::World:
            return storage().world_fixed;
        case Frame::Robot:
            return storage().robot;
    }
    return storage().world_fixed;
}

namespace detail {

bool setFrameNames(FrameNames names) {
    if (wasSet()) {
        FINENAV_LOG_ERROR(rclcpp::get_logger("finenav"),
            "setFrameNames() ignored — the frame names were already set to "
            "world_fixed='{}', robot='{}'.  Frames are set once, during setup, "
            "before any layer, map or plugin is created.",
            storage().world_fixed, storage().robot);
        return false;
    }

    storage() = std::move(names);
    wasSet() = true;

    FINENAV_LOG_INFO(rclcpp::get_logger("finenav"),
        "Frame names: world_fixed='{}', robot='{}'",
        storage().world_fixed, storage().robot);
    return true;
}

}  // namespace detail
}  // namespace finenav::common
