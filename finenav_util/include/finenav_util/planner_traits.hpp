// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

namespace finenav {

enum class TraitMask : uint32_t {
    // 空 (用于无输入需求，或不承诺任何额外输出)
    NONE         = 0,
    POSE         = 1 << 0,  // 位置 + 姿态 (Position + Orientation)
    VELOCITY     = 1 << 1,  // 线速度 + 角速度 (Linear + Angular)
    ACCELERATION = 1 << 2,  // 线加速度 + 角加速度
};

// ========================================================
// 主模板 (Primary Template)
// ========================================================
// 默认情况下使用 None
template <typename PlannerT>
struct PlannerTraits {
    // TODO: 这种情况应该警告，表示用户没有注册规划器
    static constexpr TraitMask REQUIRED_INPUTS    = TraitMask::NONE;
    static constexpr TraitMask GUARANTEED_OUTPUTS = TraitMask::NONE;
};

// TODO: HELP function,根据输入、输出 trait 组装轨迹

}

#define FINENAV_REGISTER_PLANNER_TRAITS(PLANNER_TYPE, INPUT_MASK, OUTPUT_MASK) \
namespace finenav {\
template <> \
struct PlannerTraits<PLANNER_TYPE> { \
    static constexpr TraitMask REQUIRED_INPUTS    = INPUT_MASK; \
    static constexpr TraitMask GUARANTEED_OUTPUTS = OUTPUT_MASK; \
}; \
}
