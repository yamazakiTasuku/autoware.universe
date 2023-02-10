
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

#include "behavior_velocity_planner/node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_planning_msgs/msg/path_with_lane_id.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

using autoware_auto_planning_msgs::msg::PathWithLaneId;
using autoware_auto_perception_msgs::msg::PredictedObjects;
using sensor_msgs::msg::PointCloud2;
using nav_msgs::msg::Odometry;
using geometry_msgs::msg::AccelWithCovarianceStamped;
using autoware_auto_mapping_msgs::msg::HADMapBin;
using autoware_auto_perception_msgs::msg::TrafficSignalArray;
using tier4_api_msgs::msg::CrosswalkStatus;
using tier4_api_msgs::msg::IntersectionStatus;
using tier4_planning_msgs::msg::VelocityLimit;
using tier4_v2x_msgs::msg::VirtualTrafficLightState;
using tier4_v2x_msgs::msg::VirtualTrafficLightStateArray;
using nav_msgs::msg::OccupancyGrid;
using TestNode = behavior_velocity_planner::BehaviorVelocityPlannerNode;
using tier4_autoware_utils::createPoint;
using tier4_autoware_utils::createQuaternion;
using tier4_autoware_utils::createTranslation;

class PubManager : public rclcpp::Node
{
public:
  PubManager() : Node("test_pub_node")
  {
    pub_path_ = create_publisher<PathWithLaneId>(
      "/behavior_velocity_planner_node/input/path_with_lane_id", 1);
    pub_obe_ =  create_publisher<PredictedObjects>(
       "/behavior_velocity_planner_node/input/dynamic_objects", 1);
    pub_pcl_ =  create_publisher<PointCloud2>(
       "/behavior_velocity_planner_node/input/no_ground_pointcloud", 1);
    pub_odo_ =  create_publisher<Odometry>(
       "/behavior_velocity_planner_node/input/vehicle_odometry", 1);
    pub_acc_ =  create_publisher<AccelWithCovarianceStamped>(
       "/behavior_velocity_planner_node/input/accel", 1);
    pub_map_ =  create_publisher<HADMapBin>(
       "/behavior_velocity_planner_node/input/vector_map", 1);
    pub_trasig_ =  create_publisher<TrafficSignalArray>(
       "/behavior_velocity_planner_node/input/traffic_signals", 1);
    pub_ecs_ =  create_publisher<CrosswalkStatus>(
       "/behavior_velocity_planner_node/input/external_crosswalk_states", 1);
    pub_eis_ =  create_publisher<IntersectionStatus>(
       "/behavior_velocity_planner_node/input/external_intersection_states", 1);
    pub_evlm_ =  create_publisher<VelocityLimit>(
       "/behavior_velocity_planner_node/input/external_velocity_limit_mps", 1);
    pub_ets_ =  create_publisher<TrafficSignalArray>(
       "/behavior_velocity_planner_node/input/external_traffic_signals", 1);
    pub_vtls_ =  create_publisher<VirtualTrafficLightStateArray>(
       "/behavior_velocity_planner_node/input/virtual_traffic_light_states", 1);
    pub_occ_ =  create_publisher<OccupancyGrid>(
       "/behavior_velocity_planner_node/input/occupancy_grid", 1);
    
  }
  rclcpp::Publisher<PathWithLaneId>::SharedPtr pub_path_;
  rclcpp::Publisher<PredictedObjects>::SharedPtr pub_obe_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_pcl_;
  rclcpp::Publisher<Odometry>::SharedPtr pub_odo_;
  rclcpp::Publisher<AccelWithCovarianceStamped>::SharedPtr pub_acc_;
  rclcpp::Publisher<HADMapBin>::SharedPtr pub_map_;
  rclcpp::Publisher<TrafficSignalArray>::SharedPtr pub_trasig_;
  rclcpp::Publisher<CrosswalkStatus>::SharedPtr pub_ecs_;
  rclcpp::Publisher<IntersectionStatus>::SharedPtr pub_eis_;
  rclcpp::Publisher<VelocityLimit>::SharedPtr pub_evlm_;
  rclcpp::Publisher<TrafficSignalArray>::SharedPtr pub_ets_;
  rclcpp::Publisher<VirtualTrafficLightStateArray>::SharedPtr pub_vtls_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr pub_occ_;

  void publishPathWithLaneId()
  {
    const auto now = this->now();
    PathWithLaneId pathwithlaneid;
    pathwithlaneid.header.stamp = now;
    pathwithlaneid.header.frame_id = "map";
    //pathwithlaneid.points = 
    this->pub_path_->publish(pathwithlaneid);
    rclcpp::WallRate(10).sleep();
  }

  void publishPredictedObjects()
  {
    const auto now = this->now();
    PredictedObjects predictedobjects;
    predictedobjects.header.stamp = now;
    //predictedobjects.objects
    this->pub_obe_->publish(predictedobjects);
    rclcpp::WallRate(10).sleep();
  }

  void publishPointCloud2()
  {
    const auto now = this->now();
    auto pointcloud = std::make_unique<PointCloud2>();
    //transformToBaseLink(input, header, *output_ptr);
    //this->pub_pcl_->publish(pointcloud);
    rclcpp::WallRate(10).sleep();
  }
  
   void publishOdometry()
  {
    Odometry msg;
    msg.header.stamp = get_clock()->now();
    //origin_frame_id_ = declare_parameter("origin_frame_id", "odom");
    //msg.header.frame_id = origin_frame_id_;
    //msg.child_frame_id = simulated_frame_id_;
    this->pub_odo_->publish(msg);
    rclcpp::WallRate(10).sleep();
  }
  void publishAccelWithCovarianceStamped()
  {
      AccelWithCovarianceStamped accelwithcovariancestamped;
      accelwithcovariancestamped.header.frame_id = "/base_link";
      accelwithcovariancestamped.header.stamp = get_clock()->now();
      //accelwithcovariancestamped.accel.accel.linear.x = vehicle_model_ptr_->getAx();

      using COV_IDX = tier4_autoware_utils::xyzrpy_covariance_index::XYZRPY_COV_IDX;
      constexpr auto COV = 0.001;
      accelwithcovariancestamped.accel.covariance.at(COV_IDX::X_X) = COV;          // linear x
      accelwithcovariancestamped.accel.covariance.at(COV_IDX::Y_Y) = COV;          // linear y
      accelwithcovariancestamped.accel.covariance.at(COV_IDX::Z_Z) = COV;          // linear z
      accelwithcovariancestamped.accel.covariance.at(COV_IDX::ROLL_ROLL) = COV;    // angular x
      accelwithcovariancestamped.accel.covariance.at(COV_IDX::PITCH_PITCH) = COV;  // angular y
      accelwithcovariancestamped.accel.covariance.at(COV_IDX::YAW_YAW) = COV;      // angular z
      this->pub_acc_->publish(accelwithcovariancestamped);
      rclcpp::WallRate(10).sleep();
  }
  void publishHADMapBin()
  {
    const auto lanelet2_filename = declare_parameter("lanelet2_map_path", "");
    const auto lanelet2_map_projector_type = declare_parameter("lanelet2_map_projector_type", "MGRS");
    //const auto center_line_resolution = declare_parameter("center_line_resolution", 5.0);
    // load map from file
    //const auto map = load_map(*this, lanelet2_filename, lanelet2_map_projector_type);
    // overwrite centerline
    //lanelet::utils::overwriteLaneletsCenterline(map, center_line_resolution, false);
    //const auto map_bin_msg = create_map_bin_msg(map, lanelet2_filename, now());
    //const auto map_bin_msg = create_map_bin_msg(map, lanelet2_filename, now());
    //this->pub_path_->publish(predictedobjects);
    rclcpp::WallRate(10).sleep();
  }
  void publishTrafficSignalArray()
  {
    const auto now = this->now();
    //not publisher
    rclcpp::WallRate(10).sleep();
  }
  void publishCrosswalkStatus()
  {
    const auto now = this->now();
    //not publisher
    rclcpp::WallRate(10).sleep();
  }
  void publishIntersectionStatus()
  {
    const auto now = this->now();
    //not publisher
    rclcpp::WallRate(10).sleep();
  }
  void publishVelocityLimit()
  {
    boost::optional<VelocityLimit> vel_limit;
    this->pub_evlm_->publish(vel_limit.get());
    rclcpp::WallRate(10).sleep();
  }
  void publishTrafficSignalArrayETS()
  {
    const auto now = this->now();
    //not publisher
    rclcpp::WallRate(10).sleep();
  }
  void publishVirtualTrafficLightStateArray()
  {
    VirtualTrafficLightStateArray state_array;
    VirtualTrafficLightState virtualtrafficlightState;
    state_array.stamp = get_clock()->now();
    /*
    const auto found_command_array = findCommand(
      *command_array_, node_param_.instrument_id, node_param_.use_first_command,
      node_param_.use_command_state);
    */
    this->pub_vtls_->publish(state_array);
  }
  void publishOccupancyGrid()
  {
    //this->pub_occ_->publish(OccupancyGridMapToMsgPtr(
    //map_frame_, input_raw_msg->header.stamp, pose.position.z, single_frame_occupancy_grid_map));
    rclcpp::WallRate(10).sleep();
  }

};

void declareVehicleInfoParams(rclcpp::NodeOptions & node_options)
{
  node_options.append_parameter_override("initial_engage_state", true);
  node_options.append_parameter_override("add_measurement_noise", false);
  node_options.append_parameter_override("initialize_source", "INITIAL_POSE_TOPIC");
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
    auto manager = std::make_shared<PubManager>();
    rclcpp::NodeOptions node_options;
    declareVehicleInfoParams(node_options);
    auto checker = std::make_shared<TestNode>(node_options);
    EXPECT_GE(manager->pub_path_->get_subscription_count(), 1U) << "topic is not connected.";
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(checker);
    executor.add_node(manager);
    std::thread spin_thread(
      [&executor] { ASSERT_NO_THROW(executor.spin_some()) << "error is throwed."; });
    
    testing::internal::CaptureStderr();
    manager->publishPathWithLaneId();
    ASSERT_STREQ("some_exception: Points is empty.", testing::internal::GetCapturedStderr().c_str())
      << "error output is not correct.";
    executor.cancel();
    spin_thread.join();
    
    checker.reset();
    manager.reset();
  }
}
