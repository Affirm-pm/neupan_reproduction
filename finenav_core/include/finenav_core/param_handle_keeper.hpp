// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <vector>

namespace finenav::core {

/**
 * @brief Stores shared_ptr<void> handles to prevent their managed objects
 * (typically generated ParamListener instances) from being destroyed.
 */
class ParamHandleKeeper {
public:
    template <typename T>
    void keep_alive(std::shared_ptr<T> handle) {
        if (handle) handles_.push_back(handle);
    }

private:
    std::vector<std::shared_ptr<void>> handles_;
};

}  // namespace finenav::core
