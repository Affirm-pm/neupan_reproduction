// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include <string>

#include "finenav_mppi_controller/critics/constraint_critic.hpp"
#include "finenav_mppi_controller/critics/cost_critic.hpp"
#include "finenav_mppi_controller/critics/goal_angle_critic.hpp"
#include "finenav_mppi_controller/critics/goal_critic.hpp"
#include "finenav_mppi_controller/critics/path_align_critic.hpp"
#include "finenav_mppi_controller/critics/path_angle_critic.hpp"
#include "finenav_mppi_controller/critics/path_follow_critic.hpp"
#include "finenav_mppi_controller/critics/prefer_forward_critic.hpp"
#include "finenav_mppi_controller/critics/twirling_critic.hpp"
#include "finenav_mppi_controller/critics/velocity_deadband_critic.hpp"

namespace mppi::critics
{
namespace
{

template<typename CriticT>
class TestCritic : public CriticT
{
public:
  std::string loggerName() const
  {
    return this->logger_.get_name();
  }
};

template<typename CriticT>
void expectLoggerName(
  const finenav_mppi_controller::Params & params,
  const std::string & expected_name)
{
  TestCritic<CriticT> critic;
  critic.initialize(params);
  EXPECT_EQ(critic.loggerName(), expected_name);
}

TEST(CriticFunctionTest, InitializeAssignsClassSpecificLogger)
{
  finenav_mppi_controller::Params params;

  expectLoggerName<ConstraintCritic>(params, "ConstraintCritic");
  expectLoggerName<CostCritic>(params, "CostCritic");
  expectLoggerName<GoalAngleCritic>(params, "GoalAngleCritic");
  expectLoggerName<GoalCritic>(params, "GoalCritic");
  expectLoggerName<PathAlignCritic>(params, "PathAlignCritic");
  expectLoggerName<PathAngleCritic>(params, "PathAngleCritic");
  expectLoggerName<PathFollowCritic>(params, "PathFollowCritic");
  expectLoggerName<PreferForwardCritic>(params, "PreferForwardCritic");
  expectLoggerName<TwirlingCritic>(params, "TwirlingCritic");
  expectLoggerName<VelocityDeadbandCritic>(params, "VelocityDeadbandCritic");
}

}  // namespace
}  // namespace mppi::critics
