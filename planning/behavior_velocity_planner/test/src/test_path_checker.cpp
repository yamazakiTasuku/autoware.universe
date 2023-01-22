
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
#include <common/types.hpp>

#include "behavior_velocity_planner/checker/sample_test.hpp"
#include "behavior_velocity_planner/node.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>

#include <autoware_auto_perception_msgs/msg/bounding_box.hpp>
#include <autoware_auto_planning_msgs/msg/trajectory_point.hpp>
#include <autoware_auto_vehicle_msgs/msg/vehicle_kinematic_state.hpp>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>
// using behavior_path_checker::PathPlannerArrivalChecker;
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using nav_msgs::msg::Odometry;
using TestNode = behavior_velocity_planner::BehaviorVelocityPlannerNode;
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
      create_publisher<PathWithLaneId>("/behavior_velocity_planner_node/input/path_with_lane_id", 1);
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
      this->pub_path_->publish(pathwithlaneid);
      rclcpp::WallRate(10).sleep();
    }
  }
};

void declareVehicleInfoParams(rclcpp::NodeOptions & node_options)
{
  node_options.append_parameter_override("wheel_radius", 0.5);
  node_options.append_parameter_override("wheel_width", 0.2);
  node_options.append_parameter_override("wheel_base", 3.0);
  node_options.append_parameter_override("wheel_tread", 2.0);
  node_options.append_parameter_override("front_overhang", 1.0);
  node_options.append_parameter_override("rear_overhang", 1.0);
  node_options.append_parameter_override("left_overhang", 0.5);
  node_options.append_parameter_override("right_overhang", 0.5);
  node_options.append_parameter_override("vehicle_height", 1.5);
  node_options.append_parameter_override("max_steer_angle", 0.7);
  node_options.append_parameter_override("ego_nearest_dist_threshold", 3.0);
  node_options.append_parameter_override("ego_nearest_yaw_threshold", 1.046);
}

TEST(vehicle_stop_checker, isVehicleStopped)
{
  {

    autoware_auto_planning_msgs::msg::PathWithLaneId::ConstSharedPtr input_path_msg;

    
    auto manager = std::make_shared<PubManager>();
    //const auto vehicle_model_type = GetParam();
    rclcpp::NodeOptions node_options;
    
    node_options.append_parameter_override("initialize_source", "INITIAL_POSE_TOPIC");
    //node_options.append_parameter_override("vehicle_model_type", vehicle_model_type);
    node_options.append_parameter_override("initial_engage_state", true);
    node_options.append_parameter_override("add_measurement_noise", false);
    declareVehicleInfoParams(node_options);
    auto checker = std::make_shared<TestNode>(node_options);


    EXPECT_GE(manager->pub_path_->get_subscription_count(), 1U) << "topic is not connected.";
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(checker);
    executor.add_node(manager);
    std::thread spin_thread =
      std::thread(std::bind(&rclcpp::executors::SingleThreadedExecutor::spin, &executor));
    manager->publishPathWithLaneId();

    //EXPECT_THROW(checker->pathChecker(input_path_msg), std::invalid_argument);
    // manager->publishPathWithLaneId();

    // EXPECT_TRUE(
    // checker->path_planner_arrival_checker->isVehicleStopped(STOP_DURATION_THRESHOLD_0_MS));
     executor.cancel();
     spin_thread.join();
     checker.reset();
     manager.reset();
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
