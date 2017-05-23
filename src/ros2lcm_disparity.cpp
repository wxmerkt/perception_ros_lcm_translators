// Copyright 2017 Wolfgang Merkt

// ros2lcm translator for disparity images

// ### ROS
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <stereo_msgs/DisparityImage.h>

// ### Standard includes
#include <algorithm>
#include <iostream>
#include <map>
#include <string>
#include <vector>

// ### LCM
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/image_t.hpp>

// ### zlib
#include <zlib.h>

class App {
 public:
  explicit App(ros::NodeHandle node_);
  ~App();

 private:
  lcm::LCM lcmPublish_;
  ros::NodeHandle node_;

  ros::Subscriber disparityImageSub_;

  void disparityImageCallback(const stereo_msgs::DisparityImageConstPtr& msg);
  void publishDisparityImage(const stereo_msgs::DisparityImageConstPtr& msg,
                             std::string channel);

  bool compress_;
  std::string lcmChannel_;

  int depth_compress_buf_size_ = 480 * 640 * sizeof(int8_t) * 10;
  uint8_t* depth_compress_buf_ = (uint8_t*)malloc(depth_compress_buf_size_);
};

App::App(ros::NodeHandle node_) : compress_(true) {
  ROS_INFO("Initializing Translator");
  if (!lcmPublish_.good()) {
    std::cerr << "ERROR: lcm is not good()" << std::endl;
  }

  std::string cameraTopic;
  ros::NodeHandle nh_("~");
  nh_.getParam("disparity_topic", cameraTopic);
  nh_.getParam("compress", compress_);
  nh_.getParam("lcm_channel", lcmChannel_);
  if (lcmChannel_ == "") lcmChannel_ = "DISPARITY";
  ROS_INFO_STREAM("Subscribing to " << cameraTopic << " and publishing on "
                                    << lcmChannel_);
  ROS_INFO_STREAM("Compress: " << static_cast<int>(compress_));
  disparityImageSub_ =
      node_.subscribe(cameraTopic, 1, &App::disparityImageCallback, this);
}

App::~App() {}

int disparity_image_counter = 0;
void App::disparityImageCallback(
    const stereo_msgs::DisparityImageConstPtr& msg) {
  if (disparity_image_counter % 80 == 0) {
    ROS_ERROR("DISPARITY [%d]", disparity_image_counter);
  }
  disparity_image_counter++;
  publishDisparityImage(msg, lcmChannel_);
}

void App::publishDisparityImage(const stereo_msgs::DisparityImageConstPtr& msg,
                                std::string channel) {
  int64_t current_utime =
      static_cast<int64_t>(std::floor(msg->header.stamp.toNSec() / 1000));

  bot_core::image_t lcm_image;
  lcm_image.pixelformat = bot_core::image_t::PIXEL_FORMAT_ANY;
  lcm_image.row_stride = msg->image.width;
  lcm_image.nmetadata = 0;
  lcm_image.utime = current_utime;

  // Need to convert to 16UC1 as we assume Multisense-like disparity images
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg->image, "16UC1");
  cv_ptr->image = cv_ptr->image / msg->delta_d;

  int n_colors = 2;  // 2 bytes per pixel
  int isize = msg->image.width * msg->image.height;

  if (compress_) {
    int uncompressed_size = n_colors * isize;
    unsigned long compressed_size = depth_compress_buf_size_;
    compress2(depth_compress_buf_, &compressed_size,
              (const Bytef*)cv_ptr->image.data, uncompressed_size,
              Z_BEST_SPEED);
    lcm_image.data.resize(compressed_size);
    memcpy(&lcm_image.data[0], depth_compress_buf_, compressed_size);
    lcm_image.size = compressed_size;
  } else {
    lcm_image.data.resize(n_colors * isize);
    memcpy(&lcm_image.data[0], cv_ptr->image.data, n_colors * isize);
    lcm_image.size = n_colors * isize;
  }
  // lcm_image.pixelformat is not used for disparity images
  lcmPublish_.publish(channel.c_str(), &lcm_image);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros2lcm_disparity");
  ros::NodeHandle nh;
  new App(nh);
  ROS_ERROR("ROS2LCM Camera Translator Ready");
  ros::spin();
  return 0;
}
