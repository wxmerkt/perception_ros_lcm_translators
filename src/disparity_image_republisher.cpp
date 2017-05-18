// Copyright 2017 Wolfgang Merkt

// Republish disparity image as image only component

#include <ros/ros.h>
#include <stereo_msgs/DisparityImage.h>
#include <iostream>

class App {
 public:
  explicit App(ros::NodeHandle node_in);
  ~App();

 private:
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher pub;
  void disparity_image_cb(const stereo_msgs::DisparityImageConstPtr& image);
};

App::App(ros::NodeHandle node_in) : nh(node_in) {
  pub = nh.advertise<sensor_msgs::Image>("/camera/pure_disparity", 1);
  sub = nh.subscribe("/camera/disparity", 1, &App::disparity_image_cb, this);
}

int disparity_image_cnt = 0;
void App::disparity_image_cb(const stereo_msgs::DisparityImageConstPtr& image) {
  disparity_image_cnt++;
  if (disparity_image_cnt % 15 == 0)
    ROS_INFO_STREAM("Total received images: " << disparity_image_cnt);
  pub.publish(image->image);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "disparity_image_republisher");
  ROS_INFO("DisparityImage to Image Republisher");
  ros::NodeHandle nh;
  new App(nh);

  ros::spin();
  return 0;
}
