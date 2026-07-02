// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#pragma once
#include <shared_mutex>
#include "finenav_core/map/map_concept.hpp"
namespace finenav {
/**
 * @brief LockedMapRO — RAII read-only view holding a shared_lock.
 *
 * Acquired via MapServer::getLockedReadView(). Construction acquires a shared
 * (read) lock; destruction releases it. Multiple concurrent readers are allowed.
 * As long as any LockedMapRO is alive, writers are blocked.
 */
template <SpatialContainer MapT>
class LockedMapRO {
public:
    LockedMapRO(const MapT& map, std::shared_mutex& mutex)
        : map_(map), lock_(mutex) {}
    const MapT* operator->() const { return &map_; }
    const MapT& get() const { return map_; }
private:
    const MapT& map_;
    std::shared_lock<std::shared_mutex> lock_;
};
/**
 * @brief LockedMapRW — RAII read-write view holding a unique_lock.
 *
 * Acquired via MapServer::getLockedWriteView(). Construction acquires an
 * exclusive (write) lock, blocking until all active LockedMapRO handles are
 * released. Use sparingly — prefer addUpdateSource() for routine data injection.
 */
template <SpatialContainer MapT>
class LockedMapRW {
public:
    LockedMapRW(MapT& map, std::shared_mutex& mutex)
        : map_(map), lock_(mutex) {}
    MapT* operator->() { return &map_; }
    MapT& get() { return map_; }
private:
    MapT& map_;
    std::unique_lock<std::shared_mutex> lock_;
};
} // namespace finenav
