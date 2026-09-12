// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_core/worldview/detail/world_view_registry.hpp"

#include "finenav_util/common/log_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace finenav::core::detail {

// Lazy function-local static defined in finenav_core.so — the single shared
// WorldView table across every plugin DSO (see FINENAV_DEFINE_REGISTRY_SINGLETON).
FINENAV_DEFINE_REGISTRY_SINGLETON(WorldViewRegistry)

void WorldViewRegistry::registerEntry(std::string name, Entry entry) {
    bool inserted = insertOrKeep(name, std::move(entry));
    if (!inserted) {
        FINENAV_LOG_WARN(rclcpp::get_logger("WorldViewRegistry"),
            "WorldView '{}' already registered — keeping the FIRST registrant.", name);
    } else {
        FINENAV_LOG_DEBUG(rclcpp::get_logger("WorldViewRegistry"),
            "WorldView '{}' registered.", name);
    }
}

}  // namespace finenav::core::detail
