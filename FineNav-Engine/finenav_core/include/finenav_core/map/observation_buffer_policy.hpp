// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
#pragma once

#include <cstddef>

namespace finenav::core {

/**
 * @brief ObservationBufferPolicy — buffer and timeout parameters for one sensor stream.
 *
 * Passed by value to MapRuntime::addObservationSource(), so a Map developer
 * constructs one per sensor.  A leaf header on purpose: the source that
 * consumes it lives in detail (map/detail/observation_source.hpp) and includes
 * this, never the other way round.
 */
struct ObservationBufferPolicy {
    std::size_t max_buffer_size{32};           ///< Max buffered frames (0 = unbounded)
    double observation_keep_time_sec{0.0};     ///< Max message age before eviction (0 = keep forever)
    double source_timeout_sec{0.0};            ///< Source timeout threshold (0 = disabled)
    double expected_update_rate_hz{0.0};       ///< Expected sensor rate; warns on starvation (0 = disabled)
};

}  // namespace finenav::core
