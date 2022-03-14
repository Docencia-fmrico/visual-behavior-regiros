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
#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/Twist.h"

namespace visual_behavior
{

Move::Move(const std::string& name, const BT::NodeConfiguration& config)
:BT::ActionNodeBase(name, config),counter_(0)
{
  pub_vel_ = nh_.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);
}
  
void
Move::halt()
{
  ROS_INFO("MOVING");
}

BT::NodeStatus
Move::tick()
{
  speed = getInput<struct speeds>("speed").value();

  if (counter_++ < 50)
  {
    geometry_msgs::Twist cmd;
    cmd.linear.x = speed.linear;
    cmd.angular.z = speed.angular;

    pub_vel_.publish(cmd);
    return BT::NodeStatus::RUNNING;
  }
  else
  {
    geometry_msgs::Twist cmd;
    pub_vel_.publish(cmd);

    return BT::NodeStatus::SUCCESS;
  }
}

} // namespace visual_behavior

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<visual_behavior::Move>("Move");
}