// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// Framework-internal machinery shared by the name-keyed algorithm registries.
// Not part of the public API — plugin authors reach the registries only through
// FINENAV_REGISTER_* (finenav_core/algo_register_macros.hpp).
//
// It lives in finenav_common, next to frames.hpp, because the two are the same
// thing: the framework's "one definition per process" discipline.  A registry
// table shared across plugin DSOs and a frame vocabulary shared across plugin
// DSOs fail in exactly the same way when one DSO gets its own copy — silently,
// and only under dlopen — so they are kept and read together, one layer below
// both finenav_core and finenav_visualizer.
//
// The concrete registries stay in finenav_core: their Entry types (MapEntry,
// PlanEntry, WorldViewEntry) are navigation concepts, and their `instance()`
// definitions are therefore emitted into finenav_core.so.  What is shared here is
// the discipline, not the tables.

#pragma once

#include <algorithm>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "finenav_common/i_param_pollable.hpp"

namespace finenav::common::detail {

/// Type-erased parameter loader shared by planner / map / world_view registries.
/// (node, void* instance) → IParamPollable; `instance` is cast back to the
/// concrete algorithm type inside the loader closure.
using ConfigLoader = std::function<std::shared_ptr<IParamPollable>(
    rclcpp::Node::SharedPtr, void*)>;

/**
 * @brief Header-only base for a name-keyed registry table.
 *
 * Provides thread-safe "keep-first" registration + lookup + sorted names.  A
 * concrete registry derives from this (one instantiation per Entry type) and
 * supplies its own `instance()` — a NON-inline function-local static defined in
 * the DSO that owns the registry (finenav_core.so for the three algorithm
 * registries) via FINENAV_DEFINE_REGISTRY_SINGLETON, so every plugin DSO shares
 * ONE table.
 *
 * This base exists to capture the cross-DSO-singleton discipline + the
 * name-table semantics in one place; each registry's Entry shape, factory and
 * reverse type→name index stay in the derived class.
 */
template <typename Entry>
class RegistryTable {
public:
    /// Insert @p entry under @p name.  Returns false (keeping the FIRST
    /// registrant) if the name already exists.
    bool insertOrKeep(std::string name, Entry entry) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto [it, inserted] = entries_.emplace(std::move(name), std::move(entry));
        return inserted;
    }

    /// Copy the entry out (safe against concurrent registration rehashing).
    std::optional<Entry> lookup(std::string_view name) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = entries_.find(std::string{name});
        if (it == entries_.end()) return std::nullopt;
        return it->second;
    }

    /// Sorted list of registered names.
    std::vector<std::string> names() const {
        std::lock_guard<std::mutex> lock(mutex_);
        std::vector<std::string> out;
        out.reserve(entries_.size());
        for (const auto& [name, _] : entries_) out.push_back(name);
        std::sort(out.begin(), out.end());
        return out;
    }

protected:
    std::unordered_map<std::string, Entry> entries_;
    mutable std::mutex mutex_;
};

/**
 * @brief Defines a registry's shared cross-DSO singleton.
 *
 * Call ONCE in the .cpp of the DSO that owns the registry — finenav_core.so for
 * the three algorithm registries — and never inline in a header.  A
 * function-local static defined in one shared object is what makes every plugin
 * DSO see the same table; an inline definition in a header would instead give
 * each DSO its own table and silently break registration.
 */
#define FINENAV_DEFINE_REGISTRY_SINGLETON(RegistryClass) \
    RegistryClass& RegistryClass::instance() {           \
        static RegistryClass registry;                   \
        return registry;                                 \
    }

/// ", "-join a name list for error messages.
inline std::string joinNames(const std::vector<std::string>& names) {
    std::string joined;
    for (size_t i = 0; i < names.size(); ++i) {
        if (i) joined += ", ";
        joined += names[i];
    }
    return joined;
}

}  // namespace finenav::common::detail
