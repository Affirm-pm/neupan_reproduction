// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <vector>

namespace finenav::core {

/**
 * @brief IParamPollable -- Abstract interface for a pollable parameter handle.
 */
class IParamPollable {
public:
    virtual ~IParamPollable() = default;
    virtual void poll() = 0;
};

/**
 * @brief ParamHandleKeeper -- Owns IParamPollable handles and provides a single pollAll() entry point.
 */
class ParamHandleKeeper {
public:
    /// Take ownership of a pollable parameter handle.
    void keep_alive(std::shared_ptr<IParamPollable> handle) {
        if (handle) pollables_.push_back(std::move(handle));
    }

    /// Poll all registered handles.
    void pollAll() {
        for (auto& p : pollables_) p->poll();
    }

private:
    std::vector<std::shared_ptr<IParamPollable>> pollables_;
};

}  // namespace finenav::core
