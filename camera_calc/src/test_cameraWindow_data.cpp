#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

int show_flg = 0;

void ImageDataCb(const sensor_msgs::ImageConstPtr& msg){

  cv::namedWindow("image");

  cv_bridge::CvImagePtr cv_ptr;
  try{
    cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
  }catch(cv_bridge::Exception e){
  ROS_ERROR("cv_bridge error : %s",e.what());
  return;
  }

  cv::Mat cv_image = cv_ptr->image;
  if(show_flg == 0){
    std::cout << "width : " << cv_image.size().width << std::endl;
    std::cout << "height : " << cv_image.size().height << std::endl;

    std::cout << "centor : (" << cv_image.size().width/2 << "," << cv_image.size().height/2 << ")" << std::endl;
    show_flg = 1;
  }

  

  //cv::imshow("image",cv_image);
  cv::waitKey(3);
}



int main(int argc,char **argv){

  ros::init(argc,argv,"window_data_read");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber data_read_sub;
  ros::Publisher image_centor;

  data_read_sub = it.subscribe("/robo/camera1/image_raw",1,&ImageDataCb);
  ros::spin();

  return 0;
}
