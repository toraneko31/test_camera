#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <iostream>

class ImageConverter
{
  ros::NodeHandle nh;
  image_transport::ImageTransport it;
  image_transport::Subscriber image_sub;
  image_transport::Publisher image_pub;


public:
  ImageConverter()
    :it(nh)
    {
      image_sub = it.subscribe("/robo/camera1/image_raw",1,&ImageConverter::imageCb,this);
      image_pub = it.advertise("/image_converter_circle/output_video", 1);
      cv::namedWindow("image");
      cv::namedWindow("circle");
    }

    ~ImageConverter()
    {
      cv::destroyWindow("image");
      cv::destroyWindow("circle");
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
      cv::Mat circle_image,gray_image,gray_image2;
      circle_image = cv_image.clone();

      cv::cvtColor(cv_image,gray_image,CV_BGR2GRAY); 
      cv::GaussianBlur(gray_image,gray_image2,cv::Size(11,11),0,0);

      std::vector<cv::Vec3f> storage;
      cv::HoughCircles(gray_image2,storage,CV_HOUGH_GRADIENT,1,50,100,30);
      
      std::vector<cv::Vec3f>::iterator it = storage.begin();
      for(;it!=storage.end();++it){
        cv::Point center(cv::saturate_cast<int>((*it)[0]), cv::saturate_cast<int>((*it)[1]));
        int radius = cv::saturate_cast<int>((*it)[2]);
        cv::circle(circle_image, center, radius, cv::Scalar(0,0,255), 2);
      }
      
      cv::imshow("image",cv_image);
      cv::imshow("circle",circle_image);

      cv::waitKey(3);

      image_pub.publish(cv_ptr->toImageMsg());

    }
  };

  int main(int argc,char **argv){
    ros::init(argc,argv,"image_converter_circle");
    ImageConverter ic;
    ros::spin();
    return 0;
  }
