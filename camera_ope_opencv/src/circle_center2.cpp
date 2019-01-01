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
      //image_sub = it.subscribe("/my_robo/camera2/image_raw",1,&ImageConverter::imageCb,this);
      image_sub = it.subscribe("/robo/camera2/image_raw",1,&ImageConverter::imageCb,this);
      image_pub = it.advertise("/circle_point2/output_video", 1);
      //point_pub = it.advertise<std_msgs::Int32MultiArray>("/center_point",10);
      point_pub = nh.advertise<std_msgs::Int32MultiArray>("/center_point2",10);
      image_centor = nh.advertise<std_msgs::Int32MultiArray>("/image_centor2",10);
      //cv::namedWindow("image2");
      cv::namedWindow("red2");
      cv::namedWindow("circle2");
    }

    ~ImageConverter()
    {
      //cv::destroyWindow("image2");
      cv::namedWindow("red2");
      cv::destroyWindow("circle2");
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

      cv::imshow("red2",red);
      //cv::cvtColor(red,gray_image,CV_BGR2GRAY); 
      cv::GaussianBlur(red,gray_image2,cv::Size(11,11),0,0);

      std::vector<cv::Vec3f> storage;
      cv::HoughCircles(gray_image2,storage,CV_HOUGH_GRADIENT,1,50,100,50);
      
      std::vector<cv::Vec3f>::iterator it = storage.begin();
      std_msgs::Int32MultiArray point;
      std_msgs::Int32MultiArray image_cen;
      point.data.clear();
      image_cen.data.clear();

      int cx[100];
      int cy[100];
      int count = 0;

      point.data.push_back(2);

      image_cen.data.push_back(cv_image.size().width/2);
      image_cen.data.push_back(cv_image.size().height/2);

      for(;it!=storage.end();++it){
        cv::Point center(cv::saturate_cast<int>((*it)[0]), cv::saturate_cast<int>((*it)[1]));
        int radius = cv::saturate_cast<int>((*it)[2]);
        cx[count] = cv::saturate_cast<int>((*it)[0]);
        cy[count] = cv::saturate_cast<int>((*it)[1]);
        //std::cout << "cx =" << cx[count];
        //std::cout << ": cy =" << cy[count] << std::endl << std::endl;
        if(count == 0){
          point.data.push_back(cx[count]);
          point.data.push_back(cy[count]);
        }
        count++;

        cv::circle(circle_image, center, radius, cv::Scalar(0,0,255), 2);
      }

      //cv::imshow("image2",cv_image);
      cv::imshow("circle2",circle_image);

      cv::waitKey(3);

      image_pub.publish(cv_ptr->toImageMsg());
      point_pub.publish(point);
      image_centor.publish(image_cen);
    }
  };

  int main(int argc,char **argv){
    ros::init(argc,argv,"circle_point2");
    ImageConverter ic;
    ros::spin();
    return 0;
  }
