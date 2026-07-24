// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>

namespace finenav::core {

class IControlLayer {
public:
    virtual ~IControlLayer() = default;
    virtual void connectFrom(const std::string& parent_topic) = 0;
    std::string layerName() const { return name_; }

protected:
    explicit IControlLayer(std::string name) : name_(std::move(name)) {}
    std::string name_;
};

}  // namespace finenav::core
