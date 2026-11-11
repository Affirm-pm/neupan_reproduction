#include <gtest/gtest.h>
#include <cstring>
#include <thread>
#include "neupan_world_view.hpp"
namespace finenav::example {
struct NeuPanWorldViewTestAccess {
    static void feed(NeuPanWorldView& v,sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud) {
        std::lock_guard<std::mutex> lock(v.mutex_);v.cloud_=cloud;v.scan_.reset();
        v.received_=std::chrono::steady_clock::now();
    }
    static void transform(NeuPanWorldView& v,const geometry_msgs::msg::TransformStamped& tf) {
        ASSERT_TRUE(v.tf_buffer_->setTransform(tf,"test"));
    }
};
struct ViewTest : testing::Test {
    rclcpp::Node::SharedPtr node;
    DummyMap map;
    std::unique_ptr<NeuPanWorldView> view;
    neupan_world_view::Params cfg;
    finenav::core::RobotState state{};
    finenav::core::InputTrajectoryT<NeuPanPlanner> ref;
    static void SetUpTestSuite(){if(!rclcpp::ok()){int argc=0;rclcpp::init(argc,nullptr);}}
    static void TearDownTestSuite(){rclcpp::shutdown();}
    void SetUp() override {
        node=rclcpp::Node::make_shared("neupan_view_test");view=std::make_unique<NeuPanWorldView>(node,map);
        cfg.points_topic="/neupan_test_cloud";cfg.points_type="point_cloud2";
        cfg.scan_range_max=6;cfg.scan_range_min=.4;cfg.points_timeout=1;view->configure(cfg);
    }
    sensor_msgs::msg::PointCloud2::SharedPtr cloud(double x=.5,double y=0) {
        auto c=std::make_shared<sensor_msgs::msg::PointCloud2>();
        c->header.frame_id=finenav::common::frames().world_fixed;c->header.stamp=node->now();
        c->width=1;c->height=1;c->point_step=12;c->row_step=12;c->data.resize(12);
        for(int i=0;i<3;++i){sensor_msgs::msg::PointField f;f.name=i==0?"x":i==1?"y":"z";f.offset=4*i;f.count=1;f.datatype=f.FLOAT32;c->fields.push_back(f);}
        float xyz[]={float(x),float(y),.4f};std::memcpy(c->data.data(),xyz,12);return c;
    }
    void prepare(){view->prepare({state,ref});}
};
TEST_F(ViewTest, MissingStaleZeroTimestampAndEmptyFrameAreInvalid) {
    prepare();EXPECT_FALSE(view->observationValid());
    auto c=cloud();c->header.stamp=(node->now()-rclcpp::Duration::from_seconds(2));
    NeuPanWorldViewTestAccess::feed(*view,c);prepare();EXPECT_FALSE(view->observationValid());
    c=cloud();c->header.stamp=builtin_interfaces::msg::Time{};
    NeuPanWorldViewTestAccess::feed(*view,c);prepare();EXPECT_FALSE(view->observationValid());
    c=cloud();c->header.frame_id="";NeuPanWorldViewTestAccess::feed(*view,c);prepare();EXPECT_FALSE(view->observationValid());
}
TEST_F(ViewTest, ValidEmptyCloudIsDifferentFromMissingTransform) {
    auto c=cloud();c->width=0;c->row_step=0;c->data.clear();
    NeuPanWorldViewTestAccess::feed(*view,c);prepare();EXPECT_TRUE(view->observationValid());EXPECT_TRUE(view->collisionPoints().empty());
    c=cloud();c->header.frame_id="nonexistent_sensor";NeuPanWorldViewTestAccess::feed(*view,c);
    prepare();EXPECT_FALSE(view->observationValid());
}
TEST_F(ViewTest, CropsAroundRobotAndRetainsNearPointsForSafety) {
    state.pose.position.x=10;auto c=cloud(10.5);
    NeuPanWorldViewTestAccess::feed(*view,c);prepare();ASSERT_TRUE(view->observationValid());
    ASSERT_EQ(view->obstaclePoints().size(),1u);EXPECT_NEAR(view->obstaclePoints()[0].x,10.5,1e-5);
    c=cloud(10.36);NeuPanWorldViewTestAccess::feed(*view,c);prepare();
    EXPECT_TRUE(view->obstaclePoints().empty());EXPECT_EQ(view->collisionPoints().size(),1u);
}
TEST_F(ViewTest, UsesAcquisitionTransformRatherThanLatest) {
    const auto old=node->now()-rclcpp::Duration::from_seconds(.2);
    geometry_msgs::msg::TransformStamped tf;tf.header.frame_id=finenav::common::frames().world_fixed;
    tf.child_frame_id="sensor_history";tf.transform.rotation.w=1;tf.header.stamp=old;tf.transform.translation.x=10;
    NeuPanWorldViewTestAccess::transform(*view,tf);
    tf.header.stamp=node->now();tf.transform.translation.x=20;NeuPanWorldViewTestAccess::transform(*view,tf);
    state.pose.position.x=10;auto c=cloud(.5);c->header.stamp=old;c->header.frame_id="sensor_history";
    NeuPanWorldViewTestAccess::feed(*view,c);prepare();ASSERT_TRUE(view->observationValid());
    ASSERT_EQ(view->collisionPoints().size(),1u);EXPECT_NEAR(view->collisionPoints()[0].x,10.5,1e-5);
}
TEST_F(ViewTest, TruncationDoesNotChangeSafetyCloudAndSourceSwitchClearsCache) {
    auto c=cloud();c->width=2;c->row_step=24;c->data.resize(24);std::memcpy(c->data.data()+12,c->data.data(),12);
    cfg.max_points=1;view->configure(cfg);NeuPanWorldViewTestAccess::feed(*view,c);prepare();
    ASSERT_TRUE(view->observationValid());EXPECT_EQ(view->obstaclePoints().size(),1u);EXPECT_EQ(view->collisionPoints().size(),2u);
    cfg.points_topic="/neupan_other_cloud";view->configure(cfg);prepare();EXPECT_FALSE(view->observationValid());
}
TEST_F(ViewTest, RejectsMalformedCloudAndInvertedBounds) {
    auto c=cloud();c->data.resize(2);NeuPanWorldViewTestAccess::feed(*view,c);prepare();EXPECT_FALSE(view->observationValid());
    c=cloud();c->fields[0].datatype=sensor_msgs::msg::PointField::FLOAT64;
    NeuPanWorldViewTestAccess::feed(*view,c);prepare();EXPECT_FALSE(view->observationValid());
    cfg.scan_range_min=10;cfg.scan_range_max=1;view->configure(cfg);
    NeuPanWorldViewTestAccess::feed(*view,cloud());prepare();EXPECT_FALSE(view->observationValid());
}
TEST_F(ViewTest, ConcurrentReceiptsAndParameterUpdatesUseImmutableSnapshots) {
    const auto c=cloud();
    std::thread producer([&]{for(int i=0;i<500;++i)NeuPanWorldViewTestAccess::feed(*view,c);});
    for(int i=0;i<100;++i){cfg.max_points=i%2;view->configure(cfg);prepare();}
    producer.join();prepare();EXPECT_TRUE(view->observationValid());EXPECT_EQ(view->collisionPoints().size(),1u);
}
} // namespace finenav::example
