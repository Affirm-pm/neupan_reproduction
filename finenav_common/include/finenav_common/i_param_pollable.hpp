// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

namespace finenav::common {

/**
 * @brief IParamPollable -- Abstract interface for a pollable parameter object.
 *
 * One instance per registered algorithm wraps that algorithm's generated
 * ParamListener so dynamic parameter updates can be applied by polling.
 *
 * This interface is public because it is named in a public signature —
 * finenav_core's BuiltWorldView::params.  The batch owner that stores them
 * (detail::ParamPollableKeeper) is an implementation detail and lives in
 * finenav_common/detail/param_pollable_keeper.hpp.
 *
 * It sits in finenav_common rather than finenav_core because it is the seam
 * between the registry spine and the algorithms, with no navigation vocabulary
 * in it: it names no map, no planner and no trajectory.
 */
class IParamPollable {
public:
    virtual ~IParamPollable() = default;
    virtual void poll() = 0;
};

}  // namespace finenav::common
