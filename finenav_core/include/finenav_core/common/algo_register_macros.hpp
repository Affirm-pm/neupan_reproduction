// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/common/algo_register_details.hpp"

// Guarantees complete definitions are visible at expansion time
#include "finenav_core/plan/control_layer.hpp"
#include "finenav_core/map/map_server.hpp"

/**
 * @brief Registers a PLANNER into the FineNav-Engine framework.
 *
 * @param ALGO_TYPE Algorithm type, must satisfy finenav::core::Planner.
 * @param PARAM_NS  Human-readable identifier for this algorithm. Must match the
 *                  parameter namespace for this algorithm (the top-level key of
 *                  its .yaml parameter file).
 */
#define FINENAV_REGISTER_PLANNER(ALGO_TYPE, PARAM_NS) \
namespace finenav::core { \
template <> \
struct detail::RegistrationSlot<ALGO_TYPE> { \
    static_assert(Planner<ALGO_TYPE>, "FINENAV_REGISTER_PLANNER: ALGO_TYPE must satisfy finenav::core::Planner. ");  \
    inline static detail::PlannerSelfRegistrar<ALGO_TYPE> _algo_registrar{ \
        #PARAM_NS, detail::makeConfigLoader<ALGO_TYPE, PARAM_NS::Params, PARAM_NS::ParamListener>(#PARAM_NS)}; \
}; \
}

/**
 * @brief Registers a MAP into the FineNav-Engine framework.
 *
 * @param ALGO_TYPE Algorithm type, must satisfy finenav::core::Map.
 * @param PARAM_NS  Human-readable identifier for this algorithm. Must match the
 *                  parameter namespace for this algorithm (the top-level key of
 *                  its .yaml parameter file).
 */
#define FINENAV_REGISTER_MAP(ALGO_TYPE, PARAM_NS) \
namespace finenav::core { \
template <> \
struct detail::RegistrationSlot<ALGO_TYPE> { \
    static_assert(Map<ALGO_TYPE>, "FINENAV_REGISTER_MAP: ALGO_TYPE must satisfy finenav::core::Map."); \
    inline static detail::MapSelfRegistrar<ALGO_TYPE> _algo_registrar{ \
        #PARAM_NS, detail::makeConfigLoader<ALGO_TYPE, PARAM_NS::Params, PARAM_NS::ParamListener>(#PARAM_NS)}; \
}; \
}
