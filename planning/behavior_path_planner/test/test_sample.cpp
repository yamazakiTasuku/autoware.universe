// Copyright 2022 TIER IV, Inc.
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

#include "test_sumple.hpp"
#include "motion_utils/vehicle/vehicle_state_checker.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

#include "behavior_path_planner/turn_signal_decider.hpp"
#include "autoware_auto_planning_msgs/msg/path_point.hpp"

#include "autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp"
#include "autoware_auto_mapping_msgs/msg/had_map_bin.hpp"
#include "tier4_planning_msgs/msg/path_change_module.hpp"
#include "tier4_planning_msgs/msg/path_change_module_array.hpp"
#include "autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp"

///#include <std_msgs/bool.h>
#include <gtest/gtest.h>

#include <memory>
#include <vector>
using motion_utils::VehicleArrivalChecker;
using motion_utils::VehicleStopChecker;
//入力
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using nav_msgs::msg::OccupancyGrid;
using tf2_msgs::msg::TFMessage;
using nav_msgs::msg::Odometry;
//using std_msgs::msg::Bool;
//using tier4_planning_msgs::msg::PathChangeModule;

//出力
using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_planning_msgs::msg::Path;
using autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand;
using autoware_auto_vehicle_msgs::msg::HazardLightsCommand;
using tier4_planning_msgs::msg::PathChangeModuleArray;
using tier4_planning_msgs::msg::PathChangeModule;
//using tier4_planning_msgs::msg::PathChangeModuleArray;

using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::createQuaternion;
using tier4_autoware_utils::createTranslation;

class CheckerNode : public rclcpp::Node
{
public:
  CheckerNode() : Node("test_checker_node")
  {
    vehicle_stop_checker_ = std::make_unique<VehicleStopChecker>(this);
    vehicle_arrival_checker_ = std::make_unique<VehicleArrivalChecker>(this);
  }

  std::unique_ptr<VehicleStopChecker> vehicle_stop_checker_;
  std::unique_ptr<VehicleArrivalChecker> vehicle_arrival_checker_;
};

class PubManager : public rclcpp::Node
{
  public:
    PubManager() : Node("test_pub_node")
    {
      pub_odom_ = create_publisher<Odometry>("/localization/kinematic_state", 1);
      pub_traj_ = create_publisher<Trajectory>("/planning/scenario_planning/trajectory", 1);

    }

    rclcpp::Publisher<Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<Trajectory>::SharedPtr pub_traj_;
    
};



TEST(vehicle_stop_checker, isVehicleStopped)
{
  {
    auto checker = std::make_shared<CheckerNode>();
    auto manager = std::make_shared<PubManager>();
    EXPECT_FALSE(true)<< "topic is not connected.";
    EXPECT_GE(manager->pub_odom_->get_subscription_count(), 1U) << "topic is not connected.";

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(checker);
    executor.add_node(manager);

    executor.cancel();
    checker.reset();
    manager.reset();
  }
}
