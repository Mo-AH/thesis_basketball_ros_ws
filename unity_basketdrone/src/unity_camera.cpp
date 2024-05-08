#include <chrono>
#include <cmath>
#include <iostream>
#include <math.h>
#include <ros/ros.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32.h>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <compress_image/decompress_image.h>


using namespace std::chrono;


// Ho scritto questo nodo prendendo forte spunto da "unity_compress_subscriber.cpp"

class UnityCamera {
public:
  UnityCamera() : name_("unity_camera"), nh_("") {

    // rgb camera subscriber (get compressed image from unity)
    img_sub_ = nh_.subscribe("/unity_camera/rgb/image_raw/compressed", 1, &UnityCamera::compressImageCB, this);

    // rgb camera info subscriber 
    cam_info_sub_ = nh_.subscribe("/unity_camera/rgb/camera_info", 1, &UnityCamera::camInfoCB, this);

    // rgb camera publisher 
    img_pub_ = nh_.advertise<sensor_msgs::Image>("/unity_camera/rgb/image_raw", 1, false);

  }

protected:

  void compressImageCB(const sensor_msgs::CompressedImage::ConstPtr &msg) {

    // decompress rgb image and publish to ros (ALL THE parameterization are values in this function)
    image_msg_ = image_decompressor_.decodeImage(*msg, "unchanged");
    img_pub_.publish(image_msg_);
    //size_t image_size = image_msg_.height * image_msg_.width * 3;
    //ROS_INFO("Dimensione effettiva dell'immagine: %zu bytes", image_size);
  }


  void camInfoCB(const sensor_msgs::CameraInfo::ConstPtr &msg) {
    rgb_cam_info_msg_ = *msg;
  }


  std::string name_;
  ros::NodeHandle nh_;

  // ros::WallTimer Timer;
  ros::Publisher img_pub_;
  ros::Subscriber img_sub_, cam_info_sub_;

  // rgb camera messages
  sensor_msgs::Image image_msg_;
  sensor_msgs::CameraInfo rgb_cam_info_msg_;

  // decompress libraries objects
  decompress_image::DeCompressImage image_decompressor_;

};


int main(int argc, char **argv) {

  ros::init(argc, argv, "unity_camera");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  UnityCamera handler;
  ros::waitForShutdown();
  return 0;

}