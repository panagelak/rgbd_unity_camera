#include <chrono>
#include <cmath>
#include <compress_depth_image/decompressed_depth.h>
#include <compress_image/decompress_image.h>
#include <depth_image_to_pc2/point_cloud_xyz.h>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <vector>
// // pcl
#include <cv_bridge/cv_bridge.h>
#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

using namespace std::chrono;

class UnitySubscriber {
public:
  UnitySubscriber() : name_("unity_compress_subscriber"), nh_("") {
    // rgb camera
    img_sub_ = nh_.subscribe("/unity_camera/rgb/image_raw/compressed", 100, &UnitySubscriber::compressImageCB, this);
    img_pub_ = nh_.advertise<sensor_msgs::Image>("/unity_camera/rgb/image_raw", 0, false);

    // depth camera
    img_depth_sub_ =
        nh_.subscribe("/unity_camera/depth/image_raw/compressed", 100, &UnitySubscriber::compressDepthImageCB, this);
    img_depth_pub_ = nh_.advertise<sensor_msgs::Image>("/unity_camera/depth/image_raw", 0, false);

    // rgb and depth camera info
    cam_info_sub_ = nh_.subscribe("/unity_camera/rgb/camera_info", 100, &UnitySubscriber::camInfoCB, this);
    depth_cam_info_sub_ =
        nh_.subscribe("/unity_camera/depth/camera_info", 100, &UnitySubscriber::depthCamInfoCB, this);

    // point cloud (no color)
    // pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/unity_camera/pointcloud_out", 0, false);
  }

protected:
  void compressImageCB(const sensor_msgs::CompressedImage::ConstPtr &msg) {
    // decompress rgb image and publish to ros (ALL THE parameterization are values in this function)
    image_msg_ = image_decompressor_.decodeImage(*msg, "unchanged");
    img_pub_.publish(image_msg_);
  }
  void compressDepthImageCB(const sensor_msgs::CompressedImage::ConstPtr &msg) {
    // decompress rgb image and publish to ros
    // (ALL THE parameterization are values in this function)
    // (it does not have any parameterization for decoding)
    // however it will look the encoding field of the compressed message (coming from Unity)
    // compressedDepth png
    // compressedDepth rvl
    // compressedDepth
    // png
    depth_image_msg_ = depth_image_decompressor_.decodeDepthImage(*msg);
    img_depth_pub_.publish(depth_image_msg_);
    

    // to extract pointcloud from depth and camera info (need to check that the camera info has been received)
    // pcl pointcloud
    /*
    sensor_msgs::ImageConstPtr depth_ptr_(new sensor_msgs::Image(depth_image_msg_));
    sensor_msgs::CameraInfoConstPtr cam_info_ptr_(new sensor_msgs::CameraInfo(depth_cam_info_msg_));
    pcl_xyz_msg_ = pcl_xyz_proc_.getPointCloudXyz(depth_ptr_, cam_info_ptr_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(pcl_xyz_msg_, *pcl_cloud);
    pcl_pub_.publish(pcl_xyz_msg_);
    //*/

  }
  void camInfoCB(const sensor_msgs::CameraInfo::ConstPtr &msg) {
    rgb_cam_info_msg_ = *msg;
  }
  void depthCamInfoCB(const sensor_msgs::CameraInfo::ConstPtr &msg) {
    depth_cam_info_msg_ = *msg;
  }

  std::string name_;
  ros::NodeHandle nh_;
  // ros::WallTimer Timer;
  ros::Publisher img_pub_, img_depth_pub_, pcl_pub_;
  ros::Subscriber img_sub_, img_depth_sub_, cam_info_sub_, depth_cam_info_sub_;

  // rgb camera messages
  sensor_msgs::Image image_msg_;
  sensor_msgs::CameraInfo rgb_cam_info_msg_;
  // depth camera messages
  sensor_msgs::Image depth_image_msg_;
  sensor_msgs::CameraInfo depth_cam_info_msg_;

  // decompress libraries objects
  decompress_image::DeCompressImage image_decompressor_;
  compress_depth_image::DeCompressDepth depth_image_decompressor_;
  // point cloud constructor 
  depth_image_to_pc2::PointCloudXyz pcl_xyz_proc_;
  sensor_msgs::PointCloud2 pcl_xyz_msg_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "unity_compress_subscriber");
  ros::AsyncSpinner spinner(3);
  spinner.start();
  UnitySubscriber handler;
  ros::waitForShutdown();
  return 0;
}