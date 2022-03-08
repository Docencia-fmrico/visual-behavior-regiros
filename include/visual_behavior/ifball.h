
// Copyright 2019 Intelligent Robotics Lab
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

#ifndef VISUAL_BEHAVIOR_IFBALL_H
#define VISUAL_BEHAVIOR_IFBALL_H

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include "cameras_cpp/nodo_camera.cpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "visual_behavior/str_followobj.h"

#include "ros/ros.h"

namespace visual_behavior
{

class ifball : public BT::ActionNodeBase
{
  public:
    explicit ifball(const std::string& name, const BT::NodeConfiguration& config);

    void halt();

    BT::NodeStatus tick();

    void callback_fdp(const sensor_msgs::ImageConstPtr& depth, const sensor_msgs::ImageConstPtr& rgb);

}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<struct speeds>("speed")};
    }

  private:
    ros::NodeHandle nh_;
    ros::Subscriber depth_sub_;
    ros::Subscriber hsvf_sub_;
    struct speeds spd;
    struct objectinimage ball;
    bool detected;
   
};

}  // namespace visual_behavior

#endif  // VISUAL_BEHAVIOR_IFBALL_H