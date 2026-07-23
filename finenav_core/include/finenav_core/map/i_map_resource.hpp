// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>

namespace finenav::core {

class IMapResource {
public:
    virtual ~IMapResource() = default;
    std::string resourceName() const { return name_; }

protected:
    explicit IMapResource(std::string name) : name_(std::move(name)) {}
    std::string name_;
};

}  // namespace finenav::core
