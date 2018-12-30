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
      image_pub = it.advertise("/image_converter_test/output_video", 1);
      cv::namedWindow("image");
    }

    ~ImageConverter()
    {
      cv::destroyWindow("image");
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

      if(cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60) cv::circle(cv_ptr->image,cv::Point(50,50),10,CV_RGB(255,0,0));

      cv::imshow("image",cv_ptr->image);
      cv::waitKey(3);

      image_pub.publish(cv_ptr->toImageMsg());

    }
  };

  int main(int argc,char **argv){
    ros::init(argc,argv,"image_converter_test");
    ImageConverter ic;
    ros::spin();
    return 0;
  }
