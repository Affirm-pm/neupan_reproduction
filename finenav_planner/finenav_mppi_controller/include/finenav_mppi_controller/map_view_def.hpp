#ifndef FINENAV_MAP_VIEW_DEF_HPP
#define FINENAV_MAP_VIEW_DEF_HPP

#include "finenav_core/math_types.hpp"

namespace nav2_mppi_controller {
    class IMapView {
    public:
        IMapView() =  default;
        virtual ~IMapView() = default;
        // 是否将未知区域视为可通行
        virtual bool isTrackingUnknown() const = 0;

        virtual bool considerFootprint() const = 0;

        // virtual bool isCollision(
        //   float cost,
        //   float x, float y, float theta,
        //   bool consider_footprint) const = 0;

        // 获取地图索引处的代价值（根据项目需要可改为返回 unsigned char 等）
        virtual int getCost(const finenav::Position3D& pos ) const = 0;

        virtual int getCost(const finenav::Index& index) const = 0;

        // 足迹类型，通常为局部点集 (x,y)
        using Footprint = std::vector<std::pair<float, float>>;

        // 返回机器人当前足迹
        virtual const Footprint & getRobotFootprint() const = 0;

        // 使用足迹在给定位姿处计算代价（用于 footprint 检查）
        virtual float footprintCostAtPose(
          float x, float y, float theta,
          const Footprint & footprint) const = 0;

        virtual std::string getBaseFrameID() const = 0;
    };
}

#endif
