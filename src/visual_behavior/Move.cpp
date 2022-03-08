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

#include "visual_behavior/Move.h"
#include "ros/ros.h"

namespace visual_behavior
{

  Move::Move()
  {
    pub_vel_ = n_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
  }
  

  void Move::go()
  {
    geometry_msgs::Twist cmd;

    str_followobj::speeds spd = getInput<str_followobj::speeds>("speed").value();

    cmd.linear.x = spd.linear;
    cmd.angular.z = spd.angular;

    pub_vel_.publish(cmd);
    
  }

} // namespace visual_behavior