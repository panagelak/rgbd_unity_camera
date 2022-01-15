/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

// #include "image_transport/simple_publisher_plugin.h"
// #include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
// #include <compress_depth_image/CompressDepthConfig.h>

namespace compress_depth_image {
// public image_transport::SimplePublisherPlugin<sensor_msgs::CompressedImage>
class CompressDepth {
public:
  CompressDepth();
  ~CompressDepth();

  sensor_msgs::CompressedImage encodeDepthImage(const sensor_msgs::Image &message, const std::string &format,
                                                const double &depth_max, const double &depth_quantization,
                                                const int &png_level);

  // virtual std::string getTransportName() const {
  //   return "compressedDepth";
  // }

protected:
  ros::NodeHandle nh_;
  // Overridden to set up reconfigure server
  // virtual void advertiseImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
  //                            const image_transport::SubscriberStatusCallback  &user_connect_cb,
  //                            const image_transport::SubscriberStatusCallback  &user_disconnect_cb,
  //                            const ros::VoidPtr &tracked_object, bool latch);

  // virtual void publish(const sensor_msgs::Image& message,
  //  const PublishFn& publish_fn) const;

  // typedef compress_depth_image::CompressDepthConfig Config;
  // typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  // boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  // Config config_;

  // void configCb(Config& config, uint32_t level);
};

} // namespace compress_depth_image
