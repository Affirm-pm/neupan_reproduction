// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

// This file is included at the bottom of plan/branch_table.hpp.
// It contains the inline (header-only) method definitions for BranchTable.
// Do NOT include this file directly — include branch_table.hpp instead.

#pragma once

namespace finenav::core {

inline void BranchTable::registerNode(const std::string& s) {
    if (node_set_.count(s)) return;
    nodes_.push_back(s);
    node_set_.insert(s);
}

inline void BranchTable::appendUnique(std::vector<std::string>& list, const std::string& s) {
    for (const auto& existing : list) {
        if (existing == s) return;
    }
    list.push_back(s);
}

inline bool BranchTable::addBranch(const std::string& name, std::vector<std::string> stages) {
    if (name.empty() || branches_.count(name)) return false;
    if (stages.empty()) return false;

    std::unordered_set<std::string> seen;
    for (const auto& s : stages) {
        if (s.empty() || s == "_root") return false;   // _root is implicit, not an element
        if (seen.count(s)) return false;               // duplicate stage within a branch
        seen.insert(s);
    }

    branches_.emplace(name, stages);

    // Fold adjacent pairs into the dataflow graph (dedup'd fan-in / fan-out).
    for (size_t i = 0; i < stages.size(); ++i) {
        registerNode(stages[i]);
    }
    for (size_t i = 1; i < stages.size(); ++i) {
        appendUnique(succs_[stages[i - 1]], stages[i]);
        appendUnique(preds_[stages[i]], stages[i - 1]);
    }
    return true;
}

inline bool BranchTable::hasBranch(const std::string& name) const {
    return branches_.count(name) > 0;
}

inline const std::vector<std::string>& BranchTable::branch(const std::string& name) const {
    static const std::vector<std::string> empty;
    auto it = branches_.find(name);
    return (it != branches_.end()) ? it->second : empty;
}

inline const std::unordered_map<std::string, std::vector<std::string>>&
BranchTable::branches() const {
    return branches_;
}

inline const std::vector<std::string>& BranchTable::nodes() const {
    return nodes_;
}

inline std::vector<std::string> BranchTable::preds(const std::string& s) const {
    auto it = preds_.find(s);
    return (it != preds_.end()) ? it->second : std::vector<std::string>{};
}

inline std::vector<std::string> BranchTable::succs(const std::string& s) const {
    auto it = succs_.find(s);
    return (it != succs_.end()) ? it->second : std::vector<std::string>{};
}

inline std::vector<std::string> BranchTable::leaves() const {
    std::vector<std::string> result;
    for (const auto& name : nodes_) {
        auto it = succs_.find(name);
        if (it == succs_.end() || it->second.empty()) result.push_back(name);
    }
    return result;
}

inline bool BranchTable::hasNode(const std::string& s) const {
    return node_set_.count(s) > 0;
}

}  // namespace finenav::core
