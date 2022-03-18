// Copyright 2022 Regiros
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

#include <string>

#include "visual_behavior/ifball.h"
#include "visual_behavior/str_followobj.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace visual_behavior
{
  ifball::ifball(const std::string& name, const BT::NodeConfiguration & config)
  : BT::ActionNodeBase(name, config),
    linear_pid_(0.0, 1.0, 0.0, 1.0),
    angular_pid_(0.0, 1.0, 0.0, 1.0),
    nh_("~"),
    depth_sub_(nh_, "/camera/depth/image_raw", 1),
    hsvf_sub_(nh_, "/hsv/image_filtered", 1),
    sync_fdp_(MySyncPolicy_fdp(10), depth_sub_, hsvf_sub_)
  {
    nh_.getParam("turning_vel", turning_vel);
    nh_.getParam("dist_p", dist_p);
    nh_.getParam("max_vel_ang", max_vel_ang);
    nh_.getParam("max_vel_lin", max_vel_lin);
    sync_fdp_.registerCallback(boost::bind(&ifball::callback_fdp, this, _1, _2));
  }

  void
  ifball::halt()
  {
    ROS_INFO("ifball halt");
  }

  void
  ifball::callback_fdp(const sensor_msgs::ImageConstPtr& depth, const sensor_msgs::ImageConstPtr& hsvfilt)
  {
    int pos;
    cv_bridge::CvImagePtr img_ptr_depth;

    try
    {
      img_ptr_depth = cv_bridge::toCvCopy(*depth, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception:  %s", e.what());
      return;
    }

    detected = false;
    for (int h = 0; h < hsvfilt->height; h++)
    {
      for (int w = 0; w < hsvfilt->width; w++)
      {
        pos = (hsvfilt->step * h) + (3 * w);
        if ((hsvfilt->data[pos] != 0) && (hsvfilt->data[pos+1] != 0) && (hsvfilt->data[pos+2] != 0))
        {
          ball.y = h;
          ball.x = w;
          detected = true;
          break;
        }
        if (detected) { break; }
      }
    }
    if (detected)
    {
      ball.depth = img_ptr_depth->image.at<float>(cv::Point(ball.x, ball.y)) * 1.0f;
      if (ball.depth > dist_p) {detected = false;}
    }
  }

  BT::NodeStatus
  ifball::tick()
  {
    ROS_INFO("ifball [%d]", detected);

    if (detected)
    {
      double errlin = (ball.depth - ideal_depth_)/(dist_p-1);
      double errang = (ideal_x_ - ball.x)/320;

      speed.linear = (linear_pid_.get_output(errlin))*max_vel_lin;
      speed.angular = (angular_pid_.get_output(errang))*max_vel_ang;

      ROS_INFO("linear speed %f, angular %f", speed.linear, speed.angular);
      BT::TreeNode::setOutput("speed", speed);
     return BT::NodeStatus::SUCCESS;
    }
    else
    {
      speed.linear = 0.0;
      speed.angular = turning_vel;
      BT::TreeNode::setOutput("speed", speed);
      return BT::NodeStatus::FAILURE;
    }
  }
}  // namespace visual_behavior


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<visual_behavior::ifball>("ifball");
}
