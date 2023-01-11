
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
#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <rclcpp/rclcpp.hpp>
#include <gtest/gtest.h>
#include <memory>
#include <vector>
using behavior_path_checker::PathPlannerArrivalChecker;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using nav_msgs::msg::Odometry;


using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::createQuaternion;
using tier4_autoware_utils::createTranslation;


class CheckerNode : public rclcpp::Node
{
public:
  CheckerNode() : Node("test_checker_node")
  {
    path_planner_arrival_checker = std::make_unique<PathPlannerArrivalChecker>(this);
  }
  std::unique_ptr<PathPlannerArrivalChecker> path_planner_arrival_checker;
};

class PubManager : public rclcpp::Node
{
public:
  PubManager() : Node("test_pub_node")
  {
    pub_path_ = create_publisher<PathWithLaneId>("path_change_force", 1);
  }

  rclcpp::Publisher<PathWithLaneId>::SharedPtr pub_path_;

  void publishPathWithLaneId(const geometry_msgs::msg::Pose & pose, const double publish_duration)
  {
    const auto start_time = this->now();
    while (true) {
      const auto now = this->now();

      const auto time_diff = now - start_time;
      if (publish_duration < time_diff.seconds()) {
        break;
      }

      Odometry odometry;
      odometry.header.stamp = now;
      odometry.pose.pose = pose;
      odometry.twist.twist.linear = createTranslation(0.0, 0.0, 0.0);
      odometry.twist.twist.angular = createTranslation(0.0, 0.0, 0.0);

      rclcpp::WallRate(10).sleep();
    }
  }
};

TEST(vehicle_stop_checker, isVehicleStopped)
{
  {
    auto checker = std::make_shared<CheckerNode>();
    auto manager = std::make_shared<PubManager>();
    EXPECT_GE(manager->pub_odom_->get_subscription_count(), 1U) << "topic is not connected.";

    EXPECT_FALSE(checker->path_planner_arrival_checker->isVehicleStopped());

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(checker);
    executor.add_node(manager);
    std::thread spin_thread =
      std::thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &executor));
    manager->publishStoppedOdometry(ODOMETRY_HISTORY_500_MS);

    EXPECT_TRUE(
      checker->path_planner_arrival_checker->isVehicleStopped(STOP_DURATION_THRESHOLD_0_MS));
    executor.cancel();
    spin_thread.join();
    checker.reset();
    manager.reset();
  }
}
