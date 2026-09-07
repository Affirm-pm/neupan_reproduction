// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "finenav_core/plan/i_pipeline_node.hpp"

namespace finenav {

inline const std::string _root = "_root";

struct ControlPipeEdge {
    std::vector<std::string> chain;
};

inline ControlPipeEdge operator>>(const std::string& from,
                                  std::shared_ptr<finenav::core::IPipelineNode> to) {
    ControlPipeEdge e;
    e.chain = {from, to->layerName()};
    return e;
}

inline ControlPipeEdge operator>>(ControlPipeEdge&& e,
                                  std::shared_ptr<finenav::core::IPipelineNode> to) {
    e.chain.push_back(to->layerName());
    return std::move(e);
}

// String-only chaining for name-based assembly: the 傻瓜 user wires layers by
// their registered layer_name without ever holding a typed layer handle.
inline ControlPipeEdge operator>>(const std::string& from, const std::string& to) {
    ControlPipeEdge e;
    e.chain = {from, to};
    return e;
}

inline ControlPipeEdge operator>>(ControlPipeEdge&& e, const std::string& to) {
    e.chain.push_back(to);
    return std::move(e);
}

}  // namespace finenav
