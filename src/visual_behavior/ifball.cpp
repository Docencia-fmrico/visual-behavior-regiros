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

#include <string>

#include "visual_behavior/ifball.h"
#include "visual_behavior/str_followobj.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "visual_behavior/PIDController.hpp"

#include "ros/ros.h"

namespace visual_behavior
{

  ifball::ifball(const std::string& name, const BT::NodeConfiguration & config)
  : BT::ActionNodeBase(name, config), linear_pid_(0.0, 2.0, 0.0, 0.3), angular_pid_(0.0, 3.20, 0.0, 0.8)
  {
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_(nh_, "/camera/depth/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> hsvf_sub_(nh_, "/hsv/image_filtered", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy_fdp;
    message_filters::Synchronizer<MySyncPolicy_fdp> sync_fdp(MySyncPolicy_fdp(10), depth_sub_, hsvf_sub_);

    sync_fdp.registerCallback(boost::bind(&visual_behavior::ifball::callback_fdp, _1, _2));
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

    try{
        img_ptr_depth = cv_bridge::toCvCopy(*depth, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
       ROS_ERROR("cv_bridge exception:  %s", e.what());
       return;
    }
    
    detected = false;
    for( int h = 0; h < hsvfilt->height; h++){
        for( int w = 0; w < hsvfilt->width; w++){
            pos = hsvfilt->step * h + 3 * w;
            if(hsvfilt->data[pos] != 0   && 
               hsvfilt->data[pos+1] != 0 &&
               hsvfilt->data[pos+2] != 0){
                ball.y = h;
                ball.x = w;
                detected = true;
            }
        }
    }

    ball.depth = img_ptr_depth->image.at<float>(cv::Point(ball.x, ball.y)) * 0.001f;
    std::cerr << "ball at (" << ball.depth << "in pixel" << ball.x << ball.y << std::endl;
  }


  BT::NodeStatus
  ifball::tick()
  {   
    ROS_INFO("ifball [%d]", detected);

    if (detected)
    {
      double errlin = ball.depth - ideal_depth_ ;
      double errang = ball.x - ideal_x_ ;

      spd.linear = linear_pid_.get_output(errlin);
     spd.angular = angular_pid_.get_output(errang);

      ROS_INFO("linear speed %f, angular %f", spd.linear, spd.angular);
      BT::TreeNode::setOutput("speed", spd);
     return BT::NodeStatus::SUCCESS;
    }
    else
    {
    spd.linear = 0.0;
    spd.angular = 0.4;
    BT::TreeNode::setOutput("speed", spd);
    return BT::NodeStatus::FAILURE;
    }
  }

} // namespace visual_behavior
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<visual_behavior::ifball>("ifball");
}
