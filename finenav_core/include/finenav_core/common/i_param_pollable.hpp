// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

namespace finenav::core {

/**
 * @brief IParamPollable -- Abstract interface for a pollable parameter object.
 *
 * One instance per registered algorithm wraps that algorithm's generated
 * ParamListener so dynamic parameter updates can be applied by polling.
 *
 * This interface is public because it is named in a public signature —
 * BuiltWorldView::params.  The batch owner that stores them
 * (ParamPollableKeeper) is an implementation detail and lives in
 * common/detail/param_pollable_keeper.hpp.
 */
class IParamPollable {
public:
    virtual ~IParamPollable() = default;
    virtual void poll() = 0;
};

}  // namespace finenav::core
