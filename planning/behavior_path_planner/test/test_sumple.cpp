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



#include "motion_utils/vehicle/vehicle_state_checker.hpp"
#include "tier4_autoware_utils/geometry/geometry.hpp"
#include <rclcpp/rclcpp.hpp>

#include "behavior_path_planner/turn_signal_decider.hpp"
#include "autoware_auto_planning_msgs/msg/path_point.hpp"

#include "autoware_auto_vehicle_msgs/msg/turn_indicators_command.hpp"
#include "autoware_auto_mapping_msgs/msg/had_map_bin.hpp"
#include "tier4_planning_msgs/msg/path_change_module.hpp"
#include "tier4_planning_msgs/msg/path_change_module_array.hpp"
#include "autoware_auto_vehicle_msgs/msg/hazard_lights_command.hpp"

///#include <std_msgs/bool.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>

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



class CheckerNode : public rclcpp::Node
{
public:
  CheckerNode() : Node("test_checker_node")
  {
    path_laneId_checker_ = std::make_unique<PathWithLaneId>();
    path_checker_ = std::make_unique<Path>();
    turn_cmd_checker_ = std::make_unique<TurnIndicatorsCommand>();
    hazard_cmd_checker_ = std::make_unique<HazardLightsCommand>();
    path_shangeArray_checker_ = std::make_unique<PathChangeModuleArray>();
    path_change_checker_ = std::make_unique<PathChangeModule>();
    //path__checker_ = std::make_unique<PathChangeModuleArray>(this);
  }

  std::unique_ptr<PathWithLaneId> path_laneId_checker_;
  std::unique_ptr<Path> path_checker_;
  std::unique_ptr<TurnIndicatorsCommand> turn_cmd_checker_;
  std::unique_ptr<HazardLightsCommand> hazard_cmd_checker_;
  std::unique_ptr<PathChangeModuleArray> path_shangeArray_checker_;
  std::unique_ptr<PathChangeModule> path_change_checker_;
};

class PubManager : public rclcpp::Node
{
  public:
    PubManager() : Node("test_pub_node")
    {
      pub_lane_ = create_publisher<LaneletRoute>("/planning/mission_planning/route",1);
      pub_bin_ = create_publisher<HADMapBin>("/map/vector_map",1);
      pub_object_ = create_publisher<PredictedObjects>("/perception/object_recognition/objects",1);
      pub_grid_ = create_publisher<OccupancyGrid>("/perception/occupancy_grid_map/map",1);
      pub_message_ = create_publisher<TFMessage>("/tf",1);
      pub_odom_ = create_publisher<Odometry>("/localization/kinematic_state",1);
      //pub_bool_ = create_publisher<Bool>("path_change_approval",1);
      pub_path_ = create_publisher<PathChangeModule>("path_change_force",1);
    }

    rclcpp::Publisher<LaneletRoute>::SharedPtr pub_lane_;
    rclcpp::Publisher<HADMapBin>::SharedPtr pub_bin_;
    rclcpp::Publisher<PredictedObjects>::SharedPtr pub_object_;
    rclcpp::Publisher<OccupancyGrid>::SharedPtr pub_grid_;
    rclcpp::Publisher<TFMessage>::SharedPtr pub_message_;
    rclcpp::Publisher<Odometry>::SharedPtr pub_odom_;
    //rclcpp::Publisher<Bool>::SharedPtr pub_bool_;
    rclcpp::Publisher<PathChangeModule>::SharedPtr pub_path_;


    
};



TEST(BehaviorPathPlanningUnitTest, LengthDirectionTest)
{
  {
  auto checker = std::make_shared<CheckerNode>();
  auto manager = std::make_shared<PubManager>();
  EXPECT_GE(manager->pub_odom_->get_subscription_count(), 1U) << "topic is not connected.";
  
  }
}
