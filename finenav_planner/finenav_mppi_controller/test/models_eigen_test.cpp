#include <gtest/gtest.h>

#include <type_traits>
#include <utility>

#include "finenav_mppi_controller/models/control_sequence.hpp"
#include "finenav_mppi_controller/models/eigen_types.hpp"
#include "finenav_mppi_controller/models/path.hpp"
#include "finenav_mppi_controller/models/state.hpp"
#include "finenav_mppi_controller/models/trajectories.hpp"

namespace mppi::models {
namespace {

static_assert(std::is_same_v<decltype(std::declval<State>().vx), BatchArray>);
static_assert(std::is_same_v<decltype(std::declval<Trajectories>().x), BatchArray>);
static_assert(std::is_same_v<decltype(std::declval<ControlSequence>().vx), Vector>);
static_assert(std::is_same_v<decltype(std::declval<Path>().x), Vector>);

TEST(ModelsEigenTest, StateResetCreatesZeroedBatchArrays)
{
  State state;
  state.reset(3, 5);

  EXPECT_EQ(state.vx.rows(), 3);
  EXPECT_EQ(state.vx.cols(), 5);
  EXPECT_EQ(state.cwz.rows(), 3);
  EXPECT_EQ(state.cwz.cols(), 5);
  EXPECT_TRUE(state.vx.isZero());
  EXPECT_TRUE(state.cvy.isZero());
}

TEST(ModelsEigenTest, TrajectoriesResetCreatesZeroedBatchArrays)
{
  Trajectories trajectories;
  trajectories.reset(4, 6);

  EXPECT_EQ(trajectories.x.rows(), 4);
  EXPECT_EQ(trajectories.x.cols(), 6);
  EXPECT_TRUE(trajectories.x.isZero());
  EXPECT_TRUE(trajectories.yaws.isZero());
}

TEST(ModelsEigenTest, ControlSequenceResetCreatesZeroedVectors)
{
  ControlSequence control_sequence;
  control_sequence.reset(7);

  EXPECT_EQ(control_sequence.vx.size(), 7);
  EXPECT_EQ(control_sequence.wz.size(), 7);
  EXPECT_TRUE(control_sequence.vx.isZero());
  EXPECT_TRUE(control_sequence.vy.isZero());
}

TEST(ModelsEigenTest, PathResetCreatesZeroedVectors)
{
  Path path;
  path.reset(8);

  EXPECT_EQ(path.x.size(), 8);
  EXPECT_EQ(path.yaws.size(), 8);
  EXPECT_TRUE(path.x.isZero());
  EXPECT_TRUE(path.y.isZero());
}

}  // namespace
}  // namespace mppi::models
