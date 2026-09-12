// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_core/common/frames.hpp"

#include <utility>

#include "finenav_util/common/log_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace finenav::core {

namespace {

// Function-local statics defined in finenav_core.so — ONE instance per process,
// shared by every plugin DSO that links the library.  This is the whole point:
// see the note in frames.hpp about why these are not inline header variables.
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
}  // namespace finenav::core
