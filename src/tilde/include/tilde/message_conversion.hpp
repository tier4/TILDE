// Copyright 2021 Research Institute of Systems Planning, Inc.
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

#ifndef TILDE__MESSAGE_CONVERSION_HPP_
#define TILDE__MESSAGE_CONVERSION_HPP_

#include "tilde/message_conversion_detail.hpp"

#include <tuple>

// sensing_msgs
#include "tilde_msg/msg/stee_imu.hpp"
#include "tilde_msg/msg/stee_point_cloud2.hpp"

// geometry_msgs
#include "tilde_msg/msg/stee_polygon_stamped.hpp"
#include "tilde_msg/msg/stee_pose_stamped.hpp"
#include "tilde_msg/msg/stee_pose_with_covariance_stamped.hpp"
#include "tilde_msg/msg/stee_twist_stamped.hpp"
#include "tilde_msg/msg/stee_twist_with_covariance_stamped.hpp"

// nav_msgs
#include "tilde_msg/msg/stee_occupancy_grid.hpp"
#include "tilde_msg/msg/stee_odometry.hpp"

// autoware_auto_perception_msgs
#include "tilde_msg/msg/stee_detected_objects.hpp"
#include "tilde_msg/msg/stee_predicted_objects.hpp"
#include "tilde_msg/msg/stee_tracked_objects.hpp"
#include "tilde_msg/msg/stee_traffic_light_roi_array.hpp"
#include "tilde_msg/msg/stee_traffic_signal_array.hpp"

// autoware_auto_planning_msgs
#include "tilde_msg/msg/stee_path.hpp"
#include "tilde_msg/msg/stee_path_with_lane_id.hpp"
#include "tilde_msg/msg/stee_trajectory.hpp"

// autoware_auto_control_msgs
#include "tilde_msg/msg/stee_ackermann_control_command.hpp"
#include "tilde_msg/msg/stee_ackermann_lateral_command.hpp"
#include "tilde_msg/msg/stee_longitudinal_command.hpp"

namespace tilde
{
// define your type
using TypeTable = std::tuple<
  Pair<sensor_msgs::msg::PointCloud2, tilde_msg::msg::SteePointCloud2>,
  Pair<sensor_msgs::msg::Imu, tilde_msg::msg::SteeImu>,

  Pair<geometry_msgs::msg::PolygonStamped, tilde_msg::msg::SteePolygonStamped>,
  Pair<geometry_msgs::msg::PoseStamped, tilde_msg::msg::SteePoseStamped>,
  Pair<
    geometry_msgs::msg::PoseWithCovarianceStamped, tilde_msg::msg::SteePoseWithCovarianceStamped>,
  Pair<geometry_msgs::msg::TwistStamped, tilde_msg::msg::SteeTwistStamped>,
  Pair<
    geometry_msgs::msg::TwistWithCovarianceStamped, tilde_msg::msg::SteeTwistWithCovarianceStamped>,

  Pair<nav_msgs::msg::OccupancyGrid, tilde_msg::msg::SteeOccupancyGrid>,
  Pair<nav_msgs::msg::Odometry, tilde_msg::msg::SteeOdometry>,

  Pair<autoware_auto_perception_msgs::msg::DetectedObjects, tilde_msg::msg::SteeDetectedObjects>,
  Pair<autoware_auto_perception_msgs::msg::PredictedObjects, tilde_msg::msg::SteePredictedObjects>,
  Pair<autoware_auto_perception_msgs::msg::TrackedObjects, tilde_msg::msg::SteeTrackedObjects>,
  Pair<
    autoware_auto_perception_msgs::msg::TrafficLightRoiArray,
    tilde_msg::msg::SteeTrafficLightRoiArray>,
  Pair<
    autoware_auto_perception_msgs::msg::TrafficSignalArray, tilde_msg::msg::SteeTrafficSignalArray>,

  Pair<autoware_auto_planning_msgs::msg::PathWithLaneId, tilde_msg::msg::SteePathWithLaneId>,
  Pair<autoware_auto_planning_msgs::msg::Path, tilde_msg::msg::SteePath>,
  Pair<autoware_auto_planning_msgs::msg::Trajectory, tilde_msg::msg::SteeTrajectory>,

  Pair<
    autoware_auto_control_msgs::msg::AckermannControlCommand,
    tilde_msg::msg::SteeAckermannControlCommand>,
  Pair<
    autoware_auto_control_msgs::msg::AckermannLateralCommand,
    tilde_msg::msg::SteeAckermannLateralCommand>,
  Pair<
    autoware_auto_control_msgs::msg::LongitudinalCommand,
    tilde_msg::msg::SteeLongitudinalCommand> >;

template <typename Key>
struct _ConvertedType
{
  using type = typename Get<Key, TypeTable>::type;
};

template <typename Key>
using ConvertedMessageType = typename _ConvertedType<Key>::type;

}  // namespace tilde

#endif  // TILDE__MESSAGE_CONVERSION_HPP_
