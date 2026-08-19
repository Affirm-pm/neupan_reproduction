#ifndef FINENAV_MPPI_CONTROLLER__MODELS__EIGEN_TYPES_HPP_
#define FINENAV_MPPI_CONTROLLER__MODELS__EIGEN_TYPES_HPP_

#include <Eigen/Core>

namespace mppi::models
{

using BatchArray = Eigen::Array<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using Vector = Eigen::ArrayXf;

}  // namespace mppi::models

#endif  // FINENAV_MPPI_CONTROLLER__MODELS__EIGEN_TYPES_HPP_
