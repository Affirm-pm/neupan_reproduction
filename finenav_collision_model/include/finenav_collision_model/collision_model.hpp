// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include "finenav_core/math_types.hpp"
#include <concepts>
#include <type_traits>

namespace finenav {

/**
 * @brief 碰撞规则 Concept
 * @details 任何可被调用、接受 const Position3D& 并返回 bool 的可调用对象均满足此约束。
 */
template <typename F>
concept CollisionRule =
    std::invocable<F, const Position3D&> &&
    std::same_as<std::invoke_result_t<F, const Position3D&>, bool>;

} // namespace finenav
