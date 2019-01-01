#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"


class ImageConverter
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  image_transport::Publisher image_pub;
  ros::Publisher point_pub;
  ros::Publisher image_centor;
public:
  ImageConverter()
    :it(nh)
    {
      //image_sub = it.subscribe("/my_robo/camera1/image_raw",1,&ImageConverter::imageCb,this);
      image_sub = it.subscribe("/robo/camera1/image_raw",1,&ImageConverter::imageCb,this);
 
      image_pub = it.advertise("/contour_point1/output_video", 1);
      //point_pub = it.advertise<std_msgs::Int32MultiArray>("/center_point",10);
      point_pub = nh.advertise<std_msgs::Int32MultiArray>("/contour_point1",10);
      image_centor = nh.advertise<std_msgs::Int32MultiArray>("/image_centor1",10);



      //cv::namedWindow("image1");
      cv::namedWindow("red1");
      cv::namedWindow("circle1");
    }

    ~ImageConverter()
    {
      //cv::destroyWindow("image1");
      cv::namedWindow("red1");
      cv::destroyWindow("circle1");
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg){
      cv_bridge::CvImagePtr cv_ptr;
      try
      {
        cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
      }
      catch(cv_bridge::Exception& e)
      {
        ROS_ERROR("cv_bridge exception: %s",e.what());
        return;
      }

      cv::Mat cv_image = cv_ptr->image;
      cv::Mat circle_image,hsv_image,gray_image,gray_image2;
      circle_image = cv_image.clone();

      cv::cvtColor(cv_image,hsv_image,CV_BGR2HSV);
      cv::Mat channels[3];
      cv::split(hsv_image,channels);
 
      int width = cv_image.cols;
      int height = cv_image.rows;
 
      cv::Mat red = cv::Mat(cv::Size(width,height),CV_8UC1);
      uchar hue,sat;
 
      for(int y = 0;y < height;y++){
        for(int x = 0;x < width;x++){
          hue = channels[0].at<uchar>(y,x);
          sat = channels[1].at<uchar>(y,x);
          if((hue < 8 || hue > 168) && sat > 100)red.at<uchar>(y,x) = 0;
          else red.at<uchar>(y,x) = 255;
        }
      }
      cv::imshow("red1",red);
      //cv::cvtColor(red,gray_image,CV_BGR2GRAY); 
     red = ~red;
     //contours
      std::vector< std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hier;                                              
      cv::findContours(red,contours,hier,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
      if(contours.size() != 0){
      //contours_area MAX
      double max_area = 0;
      int m_a_contour = -1;
      for(int j=0;j < contours.size();j++){
        double area= cv::contourArea(contours.at(j));
        if(max_area < area){
          max_area = area;
          m_a_contour = j;
        }
      }
 
      cv::Point2f center;
      float radius;
      cv::minEnclosingCircle(cv::Mat(contours.at(m_a_contour)).reshape(2), center, radius);
      cv::circle(circle_image,center,radius,cv::Scalar(100,100,200), 2, CV_AA); 

      cv::imshow("circle1",circle_image);
      
      std_msgs::Int32MultiArray point;
      std_msgs::Int32MultiArray image_cen;
      point.data.clear();
      image_cen.data.clear();

      int cx = center.x;
      int cy = center.y;
      int count = 0;
      point.data.push_back(1);

      image_cen.data.push_back(cv_image.size().width/2);
      image_cen.data.push_back(cv_image.size().height/2);
      point.data.push_back(cx);
      point.data.push_back(cy);
      image_pub.publish(cv_ptr->toImageMsg());
      point_pub.publish(point);
      image_centor.publish(image_cen);
      }
     // cv::imshow("image1",cv_image);

      cv::waitKey(3);

          }
  };

  int main(int argc,char **argv){
    ros::init(argc,argv,"contour_point1");
    ImageConverter ic;
    ros::spin();
    return 0;
  }
