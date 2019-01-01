#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <cmath>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

/*void ImageDataCb(const sensor_msgs::ImageConstPtr& msg){

  cv::namedWindow("image");

  cv_bridge::CvImagePtr cv_ptr;
  try{
  cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
  }catch(cv_bridge::Exception e){
  ROS_ERROR("cv_bridge error : %s",e.what());
  return;
  }

  cv::Mat cv_image = cv_ptr->image;

//std::cout << "width : " << cv_image.size().width << std::endl;
//std::cout << "height : " << cv_image.size().height << std::endl;

//std::cout << "centor : (" << cv_image.size().width/2 << "," << cv_image.size().height/2 << ")" << std::endl;



cv::imshow("image",cv_image);
cv::waitKey(3);
}

 */

int camera_no = 0;
int object_centor1[2];
int object_centor2[2];
int camera_centor1[2];
int camera_centor2[2];
int flg = 0;
int i = 0;
/*
void getCb(const std_msgs::Int32MultiArray::ConstPtr& msg);
void getCameraCentor1(const std_msgs::Int32MultiArray::ConstPtr& msg);
void getCameraCentor2(const std_msgs::Int32MultiArray::ConstPtr& msg);
*/
class Distance
{
  ros::NodeHandle nh;
  ros::Subscriber point_get_sub1;
  ros::Subscriber point_get_sub2;
  ros::Subscriber image_centor1;
  ros::Subscriber image_centor2;
  ros::Publisher next_pose;

  public:
  Distance()
  {

    next_pose = nh.advertise<std_msgs::Float32MultiArray>("/next_pose",10);
    point_get_sub1 = nh.subscribe("/center_point1",10,&Distance::getCb,this);
    point_get_sub2 = nh.subscribe("/center_point2",10,&Distance::getCb,this);
    image_centor1 = nh.subscribe("/image_centor1",10,&Distance::getCameraCentor1,this);
    image_centor2 = nh.subscribe("/image_centor2",10,&Distance::getCameraCentor2,this);
  }
  ~Distance()
  {
  }

  void getCb(const std_msgs::Int32MultiArray::ConstPtr& msg){

    flg++;
    for(std::vector<int>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it){
      if(i == 0){
        camera_no = *it;
      }else{
        if(camera_no == 1)object_centor1[i-1] = *it;
        else if(camera_no == 2)object_centor2[i-1] = *it;
      }
      i++;
    }
    i = 0;
    if(flg > 1 && camera_centor1[0]!=0 && camera_centor2[0]!=0){
      float f = (800/2) / std::tan(1.57/2);
      float T = 0.11;
      float distx = 0.0;
      float disty1 = 0.0;
      float disty2 = 0.0;
      float disty = 0.0;
      float dist = 0.0;

      float rx = object_centor2[0] - camera_centor1[0];
      float lx = object_centor1[0] - camera_centor2[0];
      float ry = camera_centor1[1] - object_centor2[1];
      float ly = camera_centor2[1] - object_centor1[1];

      flg = 0;
      //std::cout << "x [" << rx << "," << lx << "]" << std::endl;

      /*distx = (f * T)/(std::abs(rx) + std::abs(lx));
        disty1 = (distx * std::abs(ry)) / f;
        disty2 = (distx * std::abs(ly)) / f;
        disty = (disty1+disty2) / 2;
        dist = std::sqrt(distx * distx +  disty * disty);
       */
      //std::cout << "y1 : " << disty1 << std::endl;
      //std::cout << "y2 : " << disty2 << std::endl;
      std::cout << "centor1:[" << object_centor1[0] <<","<<object_centor1[1]<<"]" << std::endl;
      std::cout << "centor2:[" << object_centor2[0] <<","<<object_centor2[1]<<"]" << std::endl;

      //std::cout << "camera:[" << camera_centor1[0] << "," << camera_centor1[1] << "]" << std::endl;


      float Z = (f * T) / (lx - rx);
      float X = (Z * lx ) / f + 0.03;
      float Y = (Z * ly ) / f;

      float dist_pow = X*X + Y*Y + Z*Z;
      dist = std::sqrt(dist_pow);

      std_msgs::Float32MultiArray pose;
      pose.data.clear();
      pose.data.push_back(Z);
      pose.data.push_back(-X);
      pose.data.push_back(-Y);
      pose.data.push_back(dist);

      std::cout << "dist : " << dist << std::endl << std::endl;    
      std::cout << "object_point : [" << Z << "," << -X << "," << -Y << "]" <<  std::endl << std::endl;

      next_pose.publish(pose);
    }

    return;
  }

  void getCameraCentor1(const std_msgs::Int32MultiArray::ConstPtr& msg){
    int k = 0;
    for(std::vector<int>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it){
      camera_centor1[k] = *it;
      k++;
    }

    return;
  }
  void getCameraCentor2(const std_msgs::Int32MultiArray::ConstPtr& msg){
    int k = 0;
    for(std::vector<int>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it){
      camera_centor2[k] = *it;
      k++;
    }
    return;
  }

};

int main(int argc,char **argv){
  ros::init(argc,argv,"camera_distance");
  Distance ds;
  ros::spin();
  return 0;
}

/*
int main(int argc,char **argv){

  ros::init(argc,argv,"camera_distance");
  ros::NodeHandle nh;
  //image_transport::ImageTransport it(nh);
  //image_transport::Subscriber data_read_sub;

  //data_read_sub = it.subscribe("/robo/camera1/image_raw",1,&ImageDataCb);
  ros::Subscriber point_get_sub1;
  ros::Subscriber point_get_sub2;
  ros::Subscriber image_centor1;
  ros::Subscriber image_centor2;


  ros::Publisher next_pose = nh.advertise<std_msgs::Int32MultiArray>("/next_pose",10);
  point_get_sub1 = nh.subscribe("/center_point1",10,&getCb);
  point_get_sub2 = nh.subscribe("/center_point2",10,&getCb);
  image_centor1 = nh.subscribe("/image_centor1",10,&getCameraCentor1);
  image_centor2 = nh.subscribe("/image_centor2",10,&getCameraCentor2);


  //point_sub = nh.subscribe("/center_point1",10,&pointCb);

  ros::spin();
  return 0;
}


void getCb(const std_msgs::Int32MultiArray::ConstPtr& msg){

  flg++;
  for(std::vector<int>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it){
    if(i == 0){
      camera_no = *it;
    }else{
      if(camera_no == 1)object_centor1[i-1] = *it;
      else if(camera_no == 2)object_centor2[i-1] = *it;
    }
    i++;
  }
  i = 0;
  if(flg > 1 && camera_centor1[0]!=0 && camera_centor2[0]!=0){
    float f = (800/2) / std::tan(1.57/2);
    float T = 0.11;
    float distx = 0.0;
    float disty1 = 0.0;
    float disty2 = 0.0;
    float disty = 0.0;
    float dist = 0.0;

    float rx = object_centor2[0] - camera_centor1[0];
    float lx = object_centor1[0] - camera_centor2[0];
    float ry = camera_centor1[1] - object_centor2[1];
    float ly = camera_centor2[1] - object_centor1[1];

    flg = 0;
    //std::cout << "x [" << rx << "," << lx << "]" << std::endl;

    //distx = (f * T)/(std::abs(rx) + std::abs(lx));
      //disty1 = (distx * std::abs(ry)) / f;
      //disty2 = (distx * std::abs(ly)) / f;
      //disty = (disty1+disty2) / 2;
      //dist = std::sqrt(distx * distx +  disty * disty);
     
    //std::cout << "y1 : " << disty1 << std::endl;
    //std::cout << "y2 : " << disty2 << std::endl;
    std::cout << "centor1:[" << object_centor1[0] <<","<<object_centor1[1]<<"]" << std::endl;
    std::cout << "centor2:[" << object_centor2[0] <<","<<object_centor2[1]<<"]" << std::endl;

    //std::cout << "camera:[" << camera_centor1[0] << "," << camera_centor1[1] << "]" << std::endl;


    float Z = (f * T) / (lx - rx);
    float X = (Z * lx ) / f + 0.03;
    float Y = (Z * ly ) / f;

    float dist_pow = X*X + Y*Y + Z*Z;
    dist = std::sqrt(dist_pow);

    std_msgs::Int32MultiArray pose;
    pose.data.push_back(Z);
    pose.data.push_back(-X);
    pose.data.push_back(Y);
    pose.data.push_back(dist);

    std::cout << "dist : " << dist << std::endl << std::endl;    
    std::cout << "object_point : [" << Z << "," << -X << "," << Y << "]" <<  std::endl << std::endl;

    next_pose.publish(pose);
  }

  return;
}

void getCameraCentor1(const std_msgs::Int32MultiArray::ConstPtr& msg){
  int k = 0;
  for(std::vector<int>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it){
    camera_centor1[k] = *it;
    k++;
  }

  return;
}
void getCameraCentor2(const std_msgs::Int32MultiArray::ConstPtr& msg){
  int k = 0;
  for(std::vector<int>::const_iterator it = msg->data.begin(); it != msg->data.end(); ++it){
    camera_centor2[k] = *it;
    k++;
  }
  return;
}


*/


