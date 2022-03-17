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

#include "visual_behavior/ifperson.h"
#include "visual_behavior/str_followobj.h"

#include "behaviortree_cpp_v3/behavior_tree.h"


#include "ros/ros.h"

namespace visual_behavior
{
  ifperson::ifperson(const std::string& name, const BT::NodeConfiguration & config)
  : BT::ActionNodeBase(name, config),
    linear_pid_(0.0, 1.0, 0.0, 1.0),
    angular_pid_(0.0, 1.0, 0.0, 1.0),
    nh_(),
    depth_sub_(nh_, "/camera/depth/image_raw", 1),
    bbx_sub_(nh_, "/darknet_ros/bounding_boxes", 1),
    sync_bbx_(MySyncPolicy_bbx(10), depth_sub_, bbx_sub_)
  {
    sync_bbx_.registerCallback(boost::bind(&ifperson::callback_bbx, this, _1, _2));
  }

  void
  ifperson::halt()
  {
    ROS_INFO("ifperson halt");
  }

  void 
  ifperson::callback_bbx(const sensor_msgs::ImageConstPtr& depth, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
  {
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
    for (const auto & box : boxes->bounding_boxes) {
      person.x = (box.xmax + box.xmin) / 2;
      person.y = (box.ymax + box.ymin) / 2;

      detected = (box.Class == "person");

      if (detected)
      {
        std::cerr << "person  "  << std::endl;
        person.depth = img_ptr_depth->image.at<float>(cv::Point(person.x, person.y)) * 1.0f;
        break;
      }
    }
    if(person.depth > 4.0 || std::isnan(person.depth) || std::isinf(person.depth))
    {
      detected=false;
    }
    if(detected)
    {
      std::cerr << "person at " << person.depth << " in pixel " << person.x << std::endl;
    }
    
  }

  BT::NodeStatus
  ifperson::tick()
  {   
    ROS_INFO("ifperson [%d]", detected);

    if (detected)
    {
      double errlin = (person.depth - ideal_depth_)/3.0 ;
      double errang = (ideal_x_ - person.x)/320;

      speed.linear = (linear_pid_.get_output(errlin))*1.0;
      speed.angular = (angular_pid_.get_output(errang))*0.5;

      ROS_INFO("linear speed %f, angular %f", speed.linear, speed.angular);
      BT::TreeNode::setOutput("speed", speed);
     return BT::NodeStatus::SUCCESS;
    }
    else
    {
    speed.linear = 0;
    speed.angular = 0.4;
    ROS_INFO("linear speed %f, angular %f", speed.linear, speed.angular);
    BT::TreeNode::setOutput("speed", speed);
    ROS_INFO("aaaaa");
    return BT::NodeStatus::FAILURE;
    }
  }

} // namespace visual_behavior
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<visual_behavior::ifperson>("ifperson");
}