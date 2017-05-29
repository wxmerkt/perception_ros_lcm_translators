// Copyright 2017 Wolfgang Merkt, Maurice Fallon

// Synchronized Stereo Translator

#include <ros/ros.h>
#include <time.h>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/stereo_camera_model.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <zlib.h>
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/images_t.hpp>
#include <opencv2/opencv.hpp>

class App {
 public:
  explicit App(ros::NodeHandle node_in);
  ~App();

 private:
  lcm::LCM lcm_publish_;
  ros::NodeHandle node_, nh_;

  // Combined Stereo Image:
  bot_core::images_t images_msg_out_;
  bot_core::image_t image_a_lcm_;
  bot_core::image_t image_b_lcm_;
  int image_a_type_, image_b_type_;
  void publishStereo(const sensor_msgs::ImageConstPtr& image_a_ros,
                     const sensor_msgs::CameraInfoConstPtr& info_a_ros,
                     const sensor_msgs::ImageConstPtr& image_b_ros,
                     const sensor_msgs::CameraInfoConstPtr& info_b_ros,
                     std::string camera_out);
  image_transport::ImageTransport it_;

  ///////////////////////////////////////////////////////////////////////////////
  void head_stereo_cb(const sensor_msgs::ImageConstPtr& image_a_ros,
                      const sensor_msgs::CameraInfoConstPtr& info_cam_a,
                      const sensor_msgs::ImageConstPtr& image_b_ros,
                      const sensor_msgs::CameraInfoConstPtr& info_cam_b);
  image_transport::SubscriberFilter image_a_ros_sub_, image_b_ros_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_a_ros_sub_,
      info_b_ros_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo,
                                    sensor_msgs::Image, sensor_msgs::CameraInfo>
      sync_;

  bool do_jpeg_compress_;
  int jpeg_quality_;
  bool do_zlib_compress_;
  int depth_compress_buf_size_;
  uint8_t* depth_compress_buf_;

  void prepImage(bot_core::image_t& lcm_image,
                 const sensor_msgs::ImageConstPtr& ros_image);
};

App::App(ros::NodeHandle node_in) : node_(node_in), nh_("~"), it_(node_in), sync_(5) {
  if (!lcm_publish_.good()) {
    std::cerr << "ERROR: lcm is not good()" << std::endl;
  }

  // TODO: get height and width from 

  // allocate space for zlib compressing depth data
  depth_compress_buf_size_ = 480 * 640 * sizeof(int8_t) * 10;
  depth_compress_buf_ = (uint8_t*)malloc(depth_compress_buf_size_);
  do_jpeg_compress_ = true;
  nh_.param<int>("jpeg_quality", jpeg_quality_, 95); // 95 is opencv default
  do_zlib_compress_ = true;

  ROS_INFO_STREAM("JPEG Quality: " << jpeg_quality_);

  std::string image_a_string, info_a_string, image_b_string, info_b_string;
  std::string head_stereo_root = "/camera";

  image_a_string = head_stereo_root + "/left/image_rect_color";
  info_a_string = head_stereo_root + "/left/camera_info";
  image_a_type_ = bot_core::images_t::LEFT;

  bool output_right_image = false;  // otherwise the disparity image
  if (output_right_image) {
    image_b_string = head_stereo_root + "/right/image_rect";
    info_b_string = head_stereo_root + "/right/camera_info";
    image_b_type_ = bot_core::images_t::RIGHT;
  } else {
    image_b_string = head_stereo_root + "/pure_disparity";
    info_b_string = head_stereo_root + "/right/camera_info";
    if (do_zlib_compress_) {
      image_b_type_ = bot_core::images_t::DISPARITY_ZIPPED;
    } else {
      image_b_type_ = bot_core::images_t::DISPARITY;
    }
  }

  std::cout << image_a_string
            << " is the image_a topic subscription [for stereo]\n";
  std::cout << image_b_string
            << " is the image_b topic subscription [for stereo]\n";
  image_a_ros_sub_.subscribe(it_, ros::names::resolve(image_a_string), 1);
  info_a_ros_sub_.subscribe(node_, ros::names::resolve(info_a_string), 1);
  image_b_ros_sub_.subscribe(it_, ros::names::resolve(image_b_string), 1);
  info_b_ros_sub_.subscribe(node_, ros::names::resolve(info_b_string), 1);
  sync_.connectInput(image_a_ros_sub_, info_a_ros_sub_, image_b_ros_sub_,
                     info_b_ros_sub_);
  sync_.registerCallback(
      boost::bind(&App::head_stereo_cb, this, _1, _2, _3, _4));

  images_msg_out_.images.push_back(image_a_lcm_);
  images_msg_out_.images.push_back(image_b_lcm_);
  images_msg_out_.image_types.push_back(image_a_type_);
  images_msg_out_.image_types.push_back(image_b_type_);
  // 0 left, 1 right, 2 DISPARITY, 3 maskzipped, 4 depth mm, 5 DISPARITY_ZIPPED,
  // 6 DEPTH_MM_ZIPPED
}

App::~App() {}

int stereo_counter = 0;
void App::head_stereo_cb(const sensor_msgs::ImageConstPtr& image_a_ros,
                         const sensor_msgs::CameraInfoConstPtr& info_a_ros,
                         const sensor_msgs::ImageConstPtr& image_b_ros,
                         const sensor_msgs::CameraInfoConstPtr& info_b_ros) {
  int64_t current_utime =
      (int64_t)floor(image_a_ros->header.stamp.toNSec() / 1000);

  publishStereo(image_a_ros, info_a_ros, image_b_ros, info_b_ros,
                "MULTISENSE_CAMERA");

  if (stereo_counter % 30 == 0) {
    ROS_ERROR("HDCAM [%d]", stereo_counter);
  }
  stereo_counter++;
}

void App::publishStereo(const sensor_msgs::ImageConstPtr& image_a_ros,
                        const sensor_msgs::CameraInfoConstPtr& info_a_ros,
                        const sensor_msgs::ImageConstPtr& image_b_ros,
                        const sensor_msgs::CameraInfoConstPtr& info_b_ros,
                        std::string camera_out) {
  prepImage(image_a_lcm_, image_a_ros);
  prepImage(image_b_lcm_, image_b_ros);

  images_msg_out_.images[0] = image_a_lcm_;
  images_msg_out_.images[1] = image_b_lcm_;

  images_msg_out_.n_images = images_msg_out_.images.size();
  images_msg_out_.utime =
      (int64_t)floor(image_a_ros->header.stamp.toNSec() / 1000);
  lcm_publish_.publish("MULTISENSE_CAMERA", &images_msg_out_);
  return;
}

void App::prepImage(bot_core::image_t& lcm_image,
                    const sensor_msgs::ImageConstPtr& ros_image) {
  int64_t current_utime =
      (int64_t)floor(ros_image->header.stamp.toNSec() / 1000);
  lcm_image.utime = current_utime;
  int isize = ros_image->width * ros_image->height;
  int n_colors;

  if ((ros_image->encoding.compare("mono8") == 0) ||
      ((ros_image->encoding.compare("rgb8") == 0) ||
       (ros_image->encoding.compare("bgr8") == 0))) {
    if (ros_image->encoding.compare("mono8") == 0) {
      n_colors = 1;
    } else if (ros_image->encoding.compare("rgb8") == 0) {
      n_colors = 3;
    } else if (ros_image->encoding.compare("bgr8") == 0) {
      n_colors = 3;
    } else {
      std::cout << "Encoding [" << ros_image->encoding << "] not supported\n";
      exit(-1);
    }

    void* bytes =
        const_cast<void*>(static_cast<const void*>(ros_image->data.data()));
    cv::Mat mat;
    if (n_colors == 1) {
      mat = cv::Mat(ros_image->height, ros_image->width, CV_8UC1, bytes,
                    n_colors * ros_image->width);
    } else if (n_colors == 3) {
      mat = cv::Mat(ros_image->height, ros_image->width, CV_8UC3, bytes,
                    n_colors * ros_image->width);
    } else {
      std::cout << "Number of colors [" << n_colors << "] not supported\n";
      exit(-1);
    }

    if (do_jpeg_compress_) {
      if (ros_image->encoding.compare("rgb8") ==
          0)  // non intuative color flip needed here
        cv::cvtColor(mat, mat, CV_BGR2RGB);

      std::vector<int> params;
      params.push_back(cv::IMWRITE_JPEG_QUALITY);
      params.push_back(jpeg_quality_);
      cv::imencode(".jpg", mat, lcm_image.data, params);
      lcm_image.size = lcm_image.data.size();
      lcm_image.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG;
    } else {
      if (ros_image->encoding.compare("bgr8") == 0)
        cv::cvtColor(mat, mat, CV_BGR2RGB);

      lcm_image.data.resize(n_colors * isize);
      memcpy(&lcm_image.data[0], mat.data, n_colors * isize);
      lcm_image.size = n_colors * isize;
      if (n_colors == 1) {
        lcm_image.pixelformat = bot_core::image_t::PIXEL_FORMAT_GRAY;
      } else if (n_colors == 3) {
        lcm_image.pixelformat = bot_core::image_t::PIXEL_FORMAT_RGB;
      } else {
        std::cout << "Number of colors [" << n_colors << "] not supported\n";
        exit(-1);
      }
    }
  } else if (ros_image->encoding.compare("mono16") == 0) {
    n_colors = 2;  // 2 bytes per pixel

    if (do_zlib_compress_) {
      int uncompressed_size = n_colors * isize;
      unsigned long compressed_size = depth_compress_buf_size_;
      compress2(depth_compress_buf_, &compressed_size,
                (const Bytef*)ros_image->data.data(), uncompressed_size,
                Z_BEST_SPEED);
      lcm_image.data.resize(compressed_size);
      memcpy(&lcm_image.data[0], depth_compress_buf_, compressed_size);
      lcm_image.size = compressed_size;
    } else {
      lcm_image.data.resize(n_colors * isize);
      memcpy(&lcm_image.data[0], ros_image->data.data(), n_colors * isize);
      lcm_image.size = n_colors * isize;
    }
    // lcm_image.pixelformat is not used for disparity images

  } else if (ros_image->encoding.compare("32FC1") == 0) {
    // Need to convert to 16UC1 as we assume Multisense-like disparity images
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(ros_image, "16UC1");
    cv_ptr->image = cv_ptr->image / 0.0625; // d, minimum increment as from ROS
    
    // cv_bridge::CvImagePtr ros_test = cv_bridge::toCvCopy(ros_image);
    // cv::imshow("ROS", ros_test->image);
    // cv::imshow("After Conversion", cv_ptr->image);
    // cv::waitKey(1);

    n_colors = 2;  // 2 bytes per pixel

    if (do_zlib_compress_) {
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

  } else {
    std::cout << ros_image->encoding << " | not understood\n";
    exit(-1);
  }

  lcm_image.width = ros_image->width;
  lcm_image.height = ros_image->height;
  lcm_image.nmetadata = 0;
  lcm_image.row_stride = n_colors * ros_image->width;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ros2lcm_stereo");
  std::string which_camera = "camera";

  ros::NodeHandle nh;
  
  ROS_INFO("Stereo Camera Translator: [%s]", which_camera.c_str());

  new App(nh);
  std::cout << "ros2lcm translator ready\n";
  ros::spin();
  return 0;
}
