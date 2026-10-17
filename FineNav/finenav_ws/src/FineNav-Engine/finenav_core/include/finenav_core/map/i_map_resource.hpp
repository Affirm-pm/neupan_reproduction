// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once
#include <memory>
#include <string>

namespace finenav::core {

class IMapResource {
public:
    virtual ~IMapResource() = default;
    std::string resourceName() const { return name_; }

    // ── Type-erased introspection for WorldView assembly ─────────────────────
    /// Concrete map type carried by this server (typeid(MapT).name()).
    virtual std::string mapTypeId() const = 0;
    /// Raw pointer to the map instance (used to build a WorldView).
    virtual const void* mapData() const = 0;
    /// Acquire a read lock on the map; the returned guard releases it on destruction.
    virtual std::shared_ptr<void> lockRead() = 0;

protected:
    explicit IMapResource(std::string name) : name_(std::move(name)) {}
    std::string name_;
};

}  // namespace finenav::core
