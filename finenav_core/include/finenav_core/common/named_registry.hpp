// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

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

#include "finenav_core/common/param_pollable_keeper.hpp"

namespace finenav::core {

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
 * finenav_core.so via FINENAV_DEFINE_REGISTRY_SINGLETON, so every plugin DSO
 * shares ONE table.
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
 * Call ONCE in a finenav_core .cpp (never inline in a header) — a function-local
 * static defined in the core shared object is what makes every plugin DSO see
 * the same table.  An inline definition in a header would instead give each DSO
 * its own table and silently break registration.
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

}  // namespace finenav::core
