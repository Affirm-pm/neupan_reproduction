// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace finenav::core {

/**
 * @brief BranchTable — the control-pipeline topology, reduced from a branch table.
 *
 * The user declares named, ordered stage sequences ("branches").  Same-named
 * stages across branches denote the SAME resource (fan-in / fan-out), so this
 * table reduces the branches into a lightweight dataflow graph:
 *
 *     preds(s) = { p | ∃ branch [..., p, s, ...] }   // fan-in (multiple in-edges)
 *     succs(s) = { q | ∃ branch [..., s, q, ...] }   // fan-out
 *     leaves() = { s | succs(s) is empty }           // out-degree 0
 *
 * "_root" is NOT a stored node: every branch's first stage implicitly consumes
 * _root (the navigation-goal injection).  The graph stores only real layer
 * names; the `connectFrom("_root")` on first stages is done by the assembler.
 *
 * A branch is inherently acyclic (a sequence has no "back to earlier" form);
 * the only legality checks are: non-empty stages, no duplicate stage within a
 * branch, and a unique branch name.
 */
class BranchTable {
public:
    // ── 写入（FineNavEngine / JSON parser）──────────────────────────────────

    /// Add one named branch.  Returns false (without modifying the table) if
    /// @p stages is empty, contains "_root" or ""/duplicate stages, or if a
    /// branch named @p name already exists.
    bool addBranch(const std::string& name, std::vector<std::string> stages);

    // ── 读取（BtNavigator / PipelineController）─────────────────────────────

    bool hasBranch(const std::string& name) const;

    /// Stages of a branch (never contains "_root").  Empty ref when unknown.
    const std::vector<std::string>& branch(const std::string& name) const;

    const std::unordered_map<std::string, std::vector<std::string>>& branches() const;

    /// All dedup'd stage names, in first-seen order.
    const std::vector<std::string>& nodes() const;

    /// Possible predecessors of a stage (fan-in).  Never contains "_root".
    std::vector<std::string> preds(const std::string& s) const;

    /// Possible successors of a stage (fan-out).
    std::vector<std::string> succs(const std::string& s) const;

    /// Out-degree-0 nodes (chassis forwarding target).
    std::vector<std::string> leaves() const;

    bool hasNode(const std::string& s) const;

private:
    void registerNode(const std::string& s);
    static void appendUnique(std::vector<std::string>& list, const std::string& s);

    std::unordered_map<std::string, std::vector<std::string>> branches_;
    std::unordered_map<std::string, std::vector<std::string>> preds_;
    std::unordered_map<std::string, std::vector<std::string>> succs_;
    std::vector<std::string> nodes_;
    std::unordered_set<std::string> node_set_;
};

// ══════════════════════════════════════════════════════════════════════════════
//  Implementation (header-only data structure)
// ══════════════════════════════════════════════════════════════════════════════

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
