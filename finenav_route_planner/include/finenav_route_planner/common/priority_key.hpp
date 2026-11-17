// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cmath>
#include <limits>

namespace finenav {

// =============================================================
// Key  —  D* Lite 开放列表使用的双分量优先级键
//
// 键比较按 (k1, k2) 字典序进行。
// 键越小  =>  优先级越高。
//
// 公式（Koenig & Likhachev, AAAI 2002）：
//   k1 = min(g(s), rhs(s)) + h(start, s) + km
//   k2 = min(g(s), rhs(s))
// =============================================================

/// 无穷大代价（表示不可达 / 未初始化）
inline constexpr double kInf = std::numeric_limits<double>::infinity();

// ── 浮点键比较所用的容差 epsilon ───────────────────────────────
inline constexpr double kKeyEpsilon = 1e-9;

/**
 * @brief D* Lite 优先级键（双分量）
 * @details 按 (k1, k2) 字典序比较，键越小优先级越高；
 *          所有比较运算均带 kKeyEpsilon 容差以避免浮点误差抖动
 */
struct Key {
    double k1 = kInf;   // 主分量：min(g(s), rhs(s)) + h(start, s) + km
    double k2 = kInf;   // 次分量：min(g(s), rhs(s))

    constexpr Key() = default;
    constexpr Key(double a, double b) noexcept : k1(a), k2(b) {}

    /// 带 epsilon 容差的相等比较。
    /// 当 |k1_a - k1_b| < epsilon 且 |k2_a - k2_b| < epsilon 时
    /// 两键视为相等。
    bool operator==(const Key& o) const noexcept {
        return std::abs(k1 - o.k1) < kKeyEpsilon &&
               std::abs(k2 - o.k2) < kKeyEpsilon;
    }

    /// 带 epsilon 容差的字典序小于比较（先比 k1，再比 k2）。
    bool operator<(const Key& o) const noexcept {
        if (std::abs(k1 - o.k1) >= kKeyEpsilon)
            return k1 < o.k1;
        if (std::abs(k2 - o.k2) >= kKeyEpsilon)
            return k2 < o.k2;
        return false;  // 在容差范围内视为相等
    }

    // 其余比较运算符均由 < 和 == 派生
    bool operator>(const Key& o)  const noexcept { return o < *this; }
    bool operator<=(const Key& o) const noexcept { return !(*this > o); }
    bool operator>=(const Key& o) const noexcept { return !(*this < o); }
    bool operator!=(const Key& o) const noexcept { return !(*this == o); }
};

}  // namespace finenav
