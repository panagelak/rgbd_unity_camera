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

#include "compress_depth_image/compress_depth.h"
#include <boost/make_shared.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>

#include "compress_depth_image/codec.h"
#include "compress_depth_image/compression_common.h"

#include <sstream>
#include <vector>

using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;

namespace compress_depth_image {

CompressDepth::CompressDepth() : nh_("") {
  ;
}
CompressDepth::~CompressDepth() {
  ;
}

// void CompressDepth::advertiseImpl(ros::NodeHandle &nh, const std::string &base_topic, uint32_t queue_size,
//                                         const image_transport::SubscriberStatusCallback &user_connect_cb,
//                                         const image_transport::SubscriberStatusCallback &user_disconnect_cb,
//                                         const ros::VoidPtr &tracked_object, bool latch)
// {
//   typedef image_transport::SimplePublisherPlugin<sensor_msgs::CompressedImage> Base;
//   Base::advertiseImpl(nh, base_topic, queue_size, user_connect_cb, user_disconnect_cb, tracked_object, latch);

//   // Set up reconfigure server for this topic
//   reconfigure_server_ = boost::make_shared<ReconfigureServer>(this->nh());
//   ReconfigureServer::CallbackType f = boost::bind(&CompressDepth::configCb, this, _1, _2);
//   reconfigure_server_->setCallback(f);
// }

// void CompressDepth::configCb(Config& config, uint32_t level)
// {
//   config_ = config;
// }

sensor_msgs::CompressedImage CompressDepth::encodeDepthImage(const sensor_msgs::Image &message,
                                                             const std::string &format, const double &depth_max,
                                                             const double &depth_quantization, const int &png_level) {
  sensor_msgs::CompressedImage::Ptr compressed_image =
      encodeCompressedDepthImage(message, format, depth_max, depth_quantization, png_level);
  return *compressed_image;

  // if (compressed_image) {
  //   publish_fn(*compressed_image);
  // }
}

} // namespace compress_depth_image
