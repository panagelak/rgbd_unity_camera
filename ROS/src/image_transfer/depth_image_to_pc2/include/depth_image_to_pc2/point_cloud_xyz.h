#pragma once 

#include <boost/thread.hpp>
#include <depth_image_to_pc2/depth_conversions.h>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/point_cloud2_iterator.h>

namespace depth_image_to_pc2 {

namespace enc = sensor_msgs::image_encodings;

class PointCloudXyz {
public:
  PointCloudXyz();
  sensor_msgs::PointCloud2 getPointCloudXyz(const sensor_msgs::ImageConstPtr &depth_msg,
                                  const sensor_msgs::CameraInfoConstPtr &info_msg);

private:
  image_geometry::PinholeCameraModel model_;
};

PointCloudXyz::PointCloudXyz() {
  ;
}

} // namespace depth_image_to_pc2
