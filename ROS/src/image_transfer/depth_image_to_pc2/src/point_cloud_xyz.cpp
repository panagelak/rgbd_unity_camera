/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include <depth_image_to_pc2/point_cloud_xyz.h>

namespace depth_image_to_pc2 {

namespace enc = sensor_msgs::image_encodings;


sensor_msgs::PointCloud2 PointCloudXyz::getPointCloudXyz(const sensor_msgs::ImageConstPtr &depth_msg,
                                                         const sensor_msgs::CameraInfoConstPtr &info_msg) {
  PointCloud::Ptr cloud_msg(new PointCloud);
  cloud_msg->header = depth_msg->header;
  cloud_msg->height = depth_msg->height;
  cloud_msg->width = depth_msg->width;
  cloud_msg->is_dense = false;
  cloud_msg->is_bigendian = false;

  sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  // Update camera model
  model_.fromCameraInfo(info_msg);

  if (depth_msg->encoding == enc::TYPE_16UC1 || depth_msg->encoding == enc::MONO16) {
    convert<uint16_t>(depth_msg, cloud_msg, model_);
  } else if (depth_msg->encoding == enc::TYPE_32FC1) {
    convert<float>(depth_msg, cloud_msg, model_);
  } else {
    ROS_ERROR("Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
    return *cloud_msg;
  }
  return *cloud_msg;
}

} // namespace depth_image_to_pc2
