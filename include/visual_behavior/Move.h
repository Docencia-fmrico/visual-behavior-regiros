
// Copyright 2022 Intelligent Robotics Lab
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

#ifndef VISUAL_BEHAVIOR_MOVE_H
#define VISUAL_BEHAVIOR_MOVE_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "visual_behavior/str_followobj.h"

namespace visual_behavior
{

class Move
{
  public:
  Move();

  void go();

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<str_followobj::speeds>("speed")};
  }

  private:

    ros::NodeHandle nh_;

    struct speeds spd;

    ros::Publisher pub_vel_;
};

}  // namespace visual_behavior

#endif  // VISUAL_BEHAVIOR_MOVE_H