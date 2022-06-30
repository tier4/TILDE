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

#include <tuple>

#include "tilde/message_conversion_detail.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tilde_msg/msg/stee_point_cloud2.hpp"

namespace tilde
{
// define your type
using TypeTable = std::tuple<
  Pair<sensor_msgs::msg::PointCloud2, tilde_msg::msg::SteePointCloud2>
>;

template<typename Key>
struct _ConvertedType
{
  using type = typename Get<Key, TypeTable>::type;
};

template<typename Key>
using ConvertedMessageType = typename _ConvertedType<Key>::type;

}  // namespace tilde

#endif  // TILDE__MESSAGE_CONVERSION_HPP_
