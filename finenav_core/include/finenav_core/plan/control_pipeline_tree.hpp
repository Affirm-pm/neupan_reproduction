// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace finenav::core {

class ControlPipelineTree {
public:
    ControlPipelineTree() {
        children_["_root"] = {};
    }

    // ── 写入（FineNavEngine / YAML parser）─────────────────────────────────

    bool addEdge(const std::string& from, const std::string& to) {
        if (parent_.count(to) && parent_[to] != from) return false;
        if (wouldCreateCycle(from, to)) return false;
        if (parent_.count(to) == 0) {
            parent_[to] = from;
        } else {
            return true;  // same edge, idempotent
        }
        children_[from].push_back(to);
        if (!children_.count(to)) children_[to] = {};
        if (to != "_root" && !node_set_.count(to))   { nodes_.push_back(to); node_set_.insert(to); }
        if (from != "_root" && !node_set_.count(from)) { nodes_.push_back(from); node_set_.insert(from); }
        return true;
    }

    // ── 读取（BtNavigator）─────────────────────────────────────────────────

    const std::vector<std::string>& nodes() const { return nodes_; }

    std::vector<std::string> leaves() const {
        std::vector<std::string> result;
        for (const auto& name : nodes_) {
            auto it = children_.find(name);
            if (it == children_.end() || it->second.empty()) result.push_back(name);
        }
        return result;
    }

    const std::string& parent(const std::string& node) const {
        static const std::string empty;
        auto it = parent_.find(node);
        return (it != parent_.end()) ? it->second : empty;
    }

    const std::vector<std::string>& children(const std::string& node) const {
        static const std::vector<std::string> empty;
        auto it = children_.find(node);
        return (it != children_.end()) ? it->second : empty;
    }

    bool hasNode(const std::string& node) const {
        return node_set_.count(node) || node == "_root";
    }

private:
    bool wouldCreateCycle(const std::string& from, const std::string& to) const {
        if (from == to) return true;
        std::string current = from;
        while (!current.empty() && current != "_root") {
            auto it = parent_.find(current);
            if (it == parent_.end()) break;
            current = it->second;
            if (current == to) return true;
        }
        return false;
    }

    std::unordered_map<std::string, std::string> parent_;
    std::unordered_map<std::string, std::vector<std::string>> children_;
    std::vector<std::string> nodes_;
    std::unordered_set<std::string> node_set_;
};

}  // namespace finenav::core
