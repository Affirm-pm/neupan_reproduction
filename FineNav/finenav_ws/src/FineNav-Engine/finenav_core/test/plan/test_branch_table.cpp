// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <algorithm>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "finenav_core/plan/branch_table.hpp"

using finenav::core::BranchTable;

namespace {

std::vector<std::string> sorted(std::vector<std::string> v) {
    std::sort(v.begin(), v.end());
    return v;
}

}  // namespace

// Multi-branch reduction: shared stage "c" fans in from a (branch "first") and
// from b (branch "second"), and fans out to d and e.
TEST(BranchTable, PredsSuccsLeaves)
{
    BranchTable t;
    ASSERT_TRUE(t.addBranch("first", {"a", "c", "d"}));
    ASSERT_TRUE(t.addBranch("second", {"b", "c", "e"}));

    // Dedup'd node list contains every stage exactly once, first-seen order.
    EXPECT_EQ(t.nodes(), (std::vector<std::string>{"a", "c", "d", "b", "e"}));

    // _root is never a stored node / predecessor.
    EXPECT_FALSE(t.hasNode("_root"));
    EXPECT_EQ(t.preds("a"), std::vector<std::string>{});  // first stage: root implicit
    EXPECT_EQ(t.preds("c"), (std::vector<std::string>{"a", "b"}));  // fan-in
    EXPECT_EQ(sorted(t.succs("b")), (std::vector<std::string>{"c"}));
    EXPECT_EQ(sorted(t.succs("c")), (std::vector<std::string>{"d", "e"}));  // fan-out
    EXPECT_EQ(sorted(t.leaves()), (std::vector<std::string>{"d", "e"}));    // out-degree 0
}

// A single-stage branch is a leaf with no preds/succs.
TEST(BranchTable, SingleStageBranchIsLeaf)
{
    BranchTable t;
    ASSERT_TRUE(t.addBranch("solo", {"my_layer"}));

    EXPECT_EQ(t.branch("solo"), (std::vector<std::string>{"my_layer"}));
    EXPECT_EQ(t.preds("my_layer"), std::vector<std::string>{});
    EXPECT_EQ(t.succs("my_layer"), std::vector<std::string>{});
    EXPECT_EQ(t.leaves(), (std::vector<std::string>{"my_layer"}));
}

// Duplicate stage within one branch is rejected.
TEST(BranchTable, RejectsDuplicateStageInBranch)
{
    BranchTable t;
    EXPECT_FALSE(t.addBranch("bad", {"a", "b", "a"}));
    EXPECT_FALSE(t.hasBranch("bad"));
    EXPECT_EQ(t.nodes().size(), 0u);
}

// Duplicate branch name is rejected; "_root"/empty stages are rejected.
TEST(BranchTable, RejectsDuplicateNameAndRootStage)
{
    BranchTable t;
    ASSERT_TRUE(t.addBranch("ok", {"a", "b"}));
    EXPECT_FALSE(t.addBranch("ok", {"a", "b"}));   // duplicate name
    EXPECT_FALSE(t.addBranch("empty", {}));        // empty stages
    EXPECT_FALSE(t.addBranch("root", {"_root", "a"}));  // _root is implicit
    EXPECT_EQ(t.branch("missing"), std::vector<std::string>{});
}

// Branch lookup / membership.
TEST(BranchTable, Lookup)
{
    BranchTable t;
    t.addBranch("b1", {"x", "y"});
    EXPECT_TRUE(t.hasBranch("b1"));
    EXPECT_FALSE(t.hasBranch("nope"));
    EXPECT_TRUE(t.hasNode("x"));
    EXPECT_TRUE(t.hasNode("y"));
    EXPECT_FALSE(t.hasNode("z"));
}
