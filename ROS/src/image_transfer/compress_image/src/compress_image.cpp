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

#include "compress_image/compress_image.h"
#include <boost/make_shared.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <sensor_msgs/image_encodings.h>

#include "compress_image/compression_common.h"

#include <sstream>
#include <vector>

// If OpenCV4
#if CV_VERSION_MAJOR > 3
#include <opencv2/imgcodecs/legacy/constants_c.h>
#endif

using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;

namespace compress_image {

CompressImage::CompressImage() : nh_("") {
  ;
}
CompressImage::~CompressImage() {
  ;
}

sensor_msgs::CompressedImage CompressImage::encodeImage(const sensor_msgs::Image &message, const std::string &format,
                                                        const int &jpeg_quality, const bool &jpeg_progressive,
                                                        const bool &jpeg_optimize, const int &jpeg_restart_interval,
                                                        const int &png_level) {
  // Compressed image message
  sensor_msgs::CompressedImage compressed;
  compressed.header = message.header;
  compressed.format = message.encoding;

  // Compression settings
  std::vector<int> params;

  // Get codec configuration
  compressionFormat encodingFormat = UNDEFINED;
  if (format == "jpeg")
    encodingFormat = JPEG;
  if (format == "png")
    encodingFormat = PNG;

  // Bit depth of image encoding
  int bitDepth = enc::bitDepth(message.encoding);
  int numChannels = enc::numChannels(message.encoding);

  switch (encodingFormat) {
  // JPEG Compression
  case JPEG: {
    params.resize(9, 0);
    params[0] = IMWRITE_JPEG_QUALITY;
    params[1] = jpeg_quality;
    params[2] = IMWRITE_JPEG_PROGRESSIVE;
    params[3] = jpeg_progressive ? 1 : 0;
    params[4] = IMWRITE_JPEG_OPTIMIZE;
    params[5] = jpeg_optimize ? 1 : 0;
    params[6] = IMWRITE_JPEG_RST_INTERVAL;
    params[7] = jpeg_restart_interval;

    // Update ros message format header
    compressed.format += "; jpeg compressed ";

    // Check input format
    if ((bitDepth == 8) || (bitDepth == 16)) {
      // Target image format
      std::string targetFormat;
      if (enc::isColor(message.encoding)) {
        // convert color images to BGR8 format
        targetFormat = "bgr8";
        compressed.format += targetFormat;
      }

      // OpenCV-ros bridge
      try {
        boost::shared_ptr<CompressImage> tracked_object;
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(message, tracked_object, targetFormat);

        // Compress image
        if (cv::imencode(".jpg", cv_ptr->image, compressed.data, params)) {

          float cRatio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize()) /
                         (float)compressed.data.size();
          ROS_DEBUG("Compressed Image Transport - Codec: jpg, Compression Ratio: 1:%.2f (%lu bytes)", cRatio,
                    compressed.data.size());
        } else {
          ROS_ERROR("cv::imencode (jpeg) failed on input image");
        }
      } catch (cv_bridge::Exception &e) {
        ROS_ERROR("%s", e.what());
      } catch (cv::Exception &e) {
        ROS_ERROR("%s", e.what());
      }

      // Publish message
      // publish_fn(compressed);
      // Return message
      return compressed;
    } else
      ROS_ERROR("Compressed Image Transport - JPEG compression requires 8/16-bit color format (input format is: %s)",
                message.encoding.c_str());

    break;
  }
    // PNG Compression
  case PNG: {
    params.resize(3, 0);
    params[0] = IMWRITE_PNG_COMPRESSION;
    params[1] = png_level;

    // Update ros message format header
    compressed.format += "; png compressed ";

    // Check input format
    if ((bitDepth == 8) || (bitDepth == 16)) {

      // Target image format
      stringstream targetFormat;
      if (enc::isColor(message.encoding)) {
        // convert color images to RGB domain
        targetFormat << "bgr" << bitDepth;
        compressed.format += targetFormat.str();
      }

      // OpenCV-ros bridge
      try {
        boost::shared_ptr<CompressImage> tracked_object;
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(message, tracked_object, targetFormat.str());

        // Compress image
        if (cv::imencode(".png", cv_ptr->image, compressed.data, params)) {

          float cRatio = (float)(cv_ptr->image.rows * cv_ptr->image.cols * cv_ptr->image.elemSize()) /
                         (float)compressed.data.size();
          ROS_DEBUG("Compressed Image Transport - Codec: png, Compression Ratio: 1:%.2f (%lu bytes)", cRatio,
                    compressed.data.size());
        } else {
          ROS_ERROR("cv::imencode (png) failed on input image");
        }
      } catch (cv_bridge::Exception &e) {
        ROS_ERROR("%s", e.what());
        return compressed;
      } catch (cv::Exception &e) {
        ROS_ERROR("%s", e.what());
        return compressed;
      }

      // Publish message
      // publish_fn(compressed);
      // Return message
      return compressed;
    } else
      ROS_ERROR(
          "Compressed Image Transport - PNG compression requires 8/16-bit encoded color format (input format is: %s)",
          message.encoding.c_str());
    break;
  }

  default:
    ROS_ERROR("Unknown compression type '%s', valid options are 'jpeg' and 'png'", format.c_str());
    break;
  }
}

} // namespace compress_image
