// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "finenav_core/plan/i_control_layer.hpp"

namespace finenav {

inline const std::string _root;

struct ControlPipeEdge {
    std::vector<std::string> chain;
};

inline ControlPipeEdge operator>>(const std::string& from,
                                  std::shared_ptr<finenav::core::IControlLayer> to) {
    ControlPipeEdge e;
    e.chain = {from, to->layerName()};
    return e;
}

inline ControlPipeEdge operator>>(ControlPipeEdge&& e,
                                  std::shared_ptr<finenav::core::IControlLayer> to) {
    e.chain.push_back(to->layerName());
    return std::move(e);
}

}  // namespace finenav
