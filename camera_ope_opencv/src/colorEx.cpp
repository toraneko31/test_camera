#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

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
      image_pub = it.advertise("/image_converter_color/output_video", 1);
      cv::namedWindow("image");
      cv::namedWindow("red_ex");
    }

    ~ImageConverter()
    {
      cv::destroyWindow("image");
      cv::namedWindow("red_ex");
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

      //if(cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60) cv::circle(cv_ptr->image,cv::Point(50,50),10,CV_RGB(255,0,0));
      cv::Mat cv_image = cv_ptr->image;
      cv::Mat hsv_image;
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
          if((hue < 8 || hue > 168) && sat > 100)red.at<uchar>(y,x) = 255;
          else red.at<uchar>(y,x) = 0;
        }
      }
      //contours
      std::vector< std::vector<cv::Point> > contours;
      std::vector<cv::Vec4i> hier;
      cv::findContours(red,contours,hier,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

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
      cv::circle(cv_image,center,radius,cv::Scalar(100,100,200), 2, CV_AA);
      cv::imshow("image",cv_image);
      cv::imshow("red_ex",red);
      cv::waitKey(3);

      image_pub.publish(cv_ptr->toImageMsg());

    }
  };

  int main(int argc,char **argv){
    ros::init(argc,argv,"image_converter_color");
    ImageConverter ic;
    ros::spin();
    return 0;
  }
