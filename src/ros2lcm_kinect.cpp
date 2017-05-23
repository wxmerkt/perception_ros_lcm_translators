// ### Standard includes
#include <algorithm>
#include <vector>

// ### ROS
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// ### LCM
#include <lcm/lcm-cpp.hpp>
#include <lcmtypes/bot_core/images_t.hpp>

// ### Image processing
#include <zlib.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

lcm::LCM* lcm_(new lcm::LCM);

// Maximum size of reserved buffer - smaller images are supported as well:
int local_img_buffer_size_ = 640 * 480 * sizeof(int16_t) * 4;
uint8_t* local_img_buffer_ =
    new uint8_t[local_img_buffer_size_];  // x4 used for zlib, x10 for jpeg in
                                          // kinect_lcm
bool compress_images;
bool flip_rgb;

void callback(const sensor_msgs::ImageConstPtr& rgb,
              const sensor_msgs::ImageConstPtr& depth) {
  int n_colors = 3;
  bot_core::image_t lcm_rgb;
  lcm_rgb.utime = static_cast<int64_t>(rgb->header.stamp.toNSec() /
                                       1000);  // from nsec to usec
  lcm_rgb.width = rgb->width;
  lcm_rgb.height = rgb->height;
  lcm_rgb.row_stride = n_colors * rgb->width;
  lcm_rgb.nmetadata = 0;
  int isize = rgb->data.size();
  if (!compress_images) {
    lcm_rgb.pixelformat = bot_core::image_t::PIXEL_FORMAT_RGB;
    lcm_rgb.size = isize;

    if (flip_rgb) {
      void* bytes =
          const_cast<void*>(static_cast<const void*>(rgb->data.data()));
      cv::Mat mat(rgb->height, rgb->width, CV_8UC3, bytes,
                  n_colors * rgb->width);
      cv::cvtColor(mat, mat, CV_BGR2RGB);
      lcm_rgb.data.resize(mat.step * mat.rows);
      std::copy(mat.data, mat.data + mat.step * mat.rows, lcm_rgb.data.begin());
    } else {
      lcm_rgb.data = rgb->data;
    }
  } else {
    // TODO(tbd): reallocate to speed?
    void* bytes = const_cast<void*>(static_cast<const void*>(rgb->data.data()));
    cv::Mat mat(rgb->height, rgb->width, CV_8UC3, bytes, n_colors * rgb->width);
    if (flip_rgb) cv::cvtColor(mat, mat, CV_BGR2RGB);

    std::vector<int> params;
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(80);

    cv::imencode(".jpg", mat, lcm_rgb.data, params);
    lcm_rgb.size = lcm_rgb.data.size();
    lcm_rgb.pixelformat = bot_core::image_t::PIXEL_FORMAT_MJPEG;
  }
  // lcm_->publish("OPENNI_RGB", &lcm_rgb);

  bot_core::image_t lcm_depth;
  lcm_depth.utime = static_cast<int64_t>(depth->header.stamp.toNSec() /
                                         1000);  // from nsec to usec
  lcm_depth.width = depth->width;
  lcm_depth.height = depth->height;
  lcm_depth.pixelformat = bot_core::image_t::PIXEL_FORMAT_ANY;
  lcm_depth.row_stride = lcm_depth.width;
  lcm_depth.nmetadata = 0;

  if (!compress_images) {
    lcm_depth.size = depth->data.size();
    lcm_depth.data = depth->data;
  } else {
    int uncompressed_size = 480 * 640 * 2;
    unsigned long compressed_size = local_img_buffer_size_;
    compress2(local_img_buffer_, &compressed_size,
              (const Bytef*)depth->data.data(), uncompressed_size,
              Z_BEST_SPEED);

    lcm_depth.data.resize(compressed_size);
    std::copy(local_img_buffer_, local_img_buffer_ + compressed_size,
              lcm_depth.data.begin());
    lcm_depth.size = compressed_size;
  }
  // lcm_->publish("OPENNI_DEPTH_ONLY", &lcm_depth);

  bot_core::images_t out;
  out.utime = static_cast<int64_t>(rgb->header.stamp.toNSec() /
                                   1000);  // from nsec to usec
  out.n_images = 2;
  out.image_types.resize(2);
  out.image_types[0] = bot_core::images_t::LEFT;
  if (compress_images)
    out.image_types[1] = bot_core::images_t::DEPTH_MM_ZIPPED;
  else
    out.image_types[1] = bot_core::images_t::DEPTH_MM;
  out.images.resize(out.n_images);
  out.images[0] = lcm_rgb;
  out.images[1] = lcm_depth;
  lcm_->publish("OPENNI_FRAME", &out);
}

int main(int argc, char** argv) {
  if (!lcm_->good()) {
    std::cerr << "ERROR: lcm is not good()" << std::endl;
  }

  ros::init(argc, argv, "ros2lcm_kinect");

  ros::NodeHandle nh;
  ros::NodeHandle nh_("~");
  std::string camera_name;
  bool use_rectified;
  nh_.param<std::string>("camera", camera_name, "/camera");
  nh_.param<bool>("rectified", use_rectified, false);
  nh_.param<bool>("compress_images", compress_images, true);
  nh_.param<bool>("flip_rgb", flip_rgb, false);

  // rgb: image_color, image_rect_color
  // depth: 32FC1: image, image_rect and 16UC1: image_raw, image_rect_raw
  image_transport::ImageTransport it(nh);
  image_transport::SubscriberFilter image1_sub, image2_sub;

  image_transport::TransportHints hintCompressed("compressed", ros::TransportHints(), nh);
  image_transport::TransportHints hintCompressedDepth("compressedDepth", ros::TransportHints(), nh);
  image_transport::TransportHints hintRaw("raw", ros::TransportHints(), nh);

  if (!use_rectified) {
    image1_sub.subscribe(it, camera_name + "/rgb/image_raw", 1, hintCompressed);
    image2_sub.subscribe(it, camera_name + "/depth/image_raw", 1, hintCompressedDepth);
  } else {
    image1_sub.subscribe(it, camera_name + "/rgb/image_rect_color", 1, hintCompressed);
    image2_sub.subscribe(it, camera_name + "/depth/image_rect", 1, hintCompressedDepth);
  }

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          sensor_msgs::Image>
      MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), image1_sub,
                                                   image2_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ROS_INFO_STREAM("ROS2LCM Kinect Translator ready for "
                  << camera_name << " and using rectified: " << use_rectified
                  << ", flip rgb: " << flip_rgb
                  << ",  and using compressed: " << compress_images);

  ros::spin();
  return 0;
}
