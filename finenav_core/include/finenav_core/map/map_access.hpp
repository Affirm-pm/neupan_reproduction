// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <stdexcept>
#include <string>
#include <typeinfo>
#include <utility>
#include <vector>

namespace finenav::core {

/**
 * @brief Erased handle to one resolved map handed to a MapView build function.
 *
 * The framework resolves the map types a MapView declares into concrete
 * MapServer instances, then passes the build function a MapAccess carrying one
 * handle per map: its concrete type (mangled name) and a pointer to the map
 * data.  The MapView author recovers the typed reference via get<MapT>().
 */
struct MapHandle {
    std::string type_name;  ///< typeid(MapT).name() — stable across plugin DSOs
    const void* data;       ///< pointer to the MapT instance (owned by MapServer)
};

/**
 * @brief Type-safe accessor for the maps a MapView depends on.
 *
 * A MapView's build function recovers each map by TYPE (never by name or index):
 * `maps.get<TemporalVoxelMap>()` returns `const TemporalVoxelMap&`.  The
 * framework already matched the type to a concrete MapServer, so a missing or
 * mistyped dependency raises a clear error instead of silently casting.
 */
class MapAccess {
public:
    explicit MapAccess(std::vector<MapHandle> handles) : handles_(std::move(handles)) {}

    /** @brief Recover a depended-on map by concrete type.  Throws if absent. */
    template <typename MapT>
    const MapT& get() const {
        for (const auto& h : handles_) {
            if (h.type_name == typeid(MapT).name()) {
                return *static_cast<const MapT*>(h.data);
            }
        }
        throw std::runtime_error(
            "MapAccess: this MapView's dependencies do not include a map of type '" +
            std::string(typeid(MapT).name()) + "'.");
    }

private:
    std::vector<MapHandle> handles_;
};

}  // namespace finenav::core
