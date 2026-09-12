// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <vector>

#include "finenav_core/common/i_param_pollable.hpp"

namespace finenav::core::detail {

/**
 * @brief ParamPollableKeeper -- Owns IParamPollable objects and provides a
 *        single pollAll() entry point for periodic dynamic-parameter updates.
 *
 * A private member of PipelineNode and MapServer, and nothing else: no user
 * ever names this type, so it is detail while the interface it stores
 * (IParamPollable) stays public.
 *
 * Declaration order matters where it is used — it must be declared BEFORE the
 * planner/map instance so it is destroyed AFTER it (the algorithm holds a
 * borrowed pointer into the visualizer its pollable owns).
 */
class ParamPollableKeeper {
public:
    /// Take ownership of a pollable parameter object.
    void keep_alive(std::shared_ptr<IParamPollable> pollable) {
        if (pollable) pollables_.push_back(std::move(pollable));
    }

    /// Poll all registered objects.
    void pollAll() {
        for (auto& p : pollables_) p->poll();
    }

private:
    std::vector<std::shared_ptr<IParamPollable>> pollables_;
};

}  // namespace finenav::core::detail
