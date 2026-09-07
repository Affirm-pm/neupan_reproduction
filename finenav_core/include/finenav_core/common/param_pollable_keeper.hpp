// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <vector>

namespace finenav::core {

/**
 * @brief IParamPollable -- Abstract interface for a pollable parameter object.
 */
class IParamPollable {
public:
    virtual ~IParamPollable() = default;
    virtual void poll() = 0;
};

/**
 * @brief ParamPollableKeeper -- Owns IParamPollable objects and provides a
 *        single pollAll() entry point for periodic dynamic-parameter updates.
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

}  // namespace finenav::core
