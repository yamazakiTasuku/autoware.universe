
// Copyright 2022 TierIV
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "behavior_velocity_planner/checker/sample_test.hpp"
#include "behavior_velocity_planner/node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <vector>
// using behavior_path_checker::PathPlannerArrivalChecker;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using nav_msgs::msg::Odometry;
using behavior_velocity_planner::BehaviorVelocityPlannerNode;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::createQuaternion;
using tier4_autoware_utils::createTranslation;

class PubManager : public rclcpp::Node
{
public:
  PubManager() : Node("test_pub_node")
  {
    pub_path_ =
      //create_publisher<Odometry>("/localization/kinematic_state", 1);
      create_publisher<PathWithLaneId>("/planning/scenario_planning/lane_driving/behavior_planning/path_with_lane_id", 1);
  }

  rclcpp::Publisher<PathWithLaneId>::SharedPtr pub_path_;

  void
  publishPathWithLaneId()  // const geometry_msgs::msg::Pose & pose, const double publish_duration)
  {
    const auto start_time = this->now();
    while (true) {
      const auto now = this->now();
      PathWithLaneId pathwithlaneid;
      pathwithlaneid.header.stamp = now;

      rclcpp::WallRate(10).sleep();
    }
  }
};

TEST(vehicle_stop_checker, isVehicleStopped)
{
  {
    rclcpp::NodeOptions options;
    const auto share_dir = ament_index_cpp::get_package_share_directory("behavior_velocity_planner");
    options.arguments({"--ros-args", "--params-file", share_dir + "/config/crosswalk.param.yaml"});
    
    auto manager = std::make_shared<PubManager>();
    EXPECT_GE(manager->pub_path_->get_subscription_count(), 1U) << "topic is not connected.";
    auto checker = std::make_shared<BehaviorVelocityPlannerNode>(options);
    // manager->publishPathWithLaneId();

    // EXPECT_TRUE(
    // checker->path_planner_arrival_checker->isVehicleStopped(STOP_DURATION_THRESHOLD_0_MS));
    // executor.cancel();
    // spin_thread.join();
    // checker.reset();
    // manager.reset();
  }
}

/*
TEST(onTrigger,pathChecker)
{
  using autoware_auto_planning_msgs::msg::PathWithLaneId;
  behavior_velocity_planner::BehaviorVelocityPlannerNode behaviorvel(const rclcpp::NodeOptions &
options = rclcpp::NodeOptions()); PathWithLaneId pathwithlaneid;

  // Empty
  EXPECT_THROW(behaviorvel.onTrigger(pathwithlaneid), std::runtime_error);
}
*/
