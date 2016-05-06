#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/videoio.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_srvs/Empty.h>
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/Image.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace std_msgs;

class DataRecorder
{
public:
  DataRecorder();

private:
  bool saveImageCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  bool startRecordingCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  bool stopRecordingCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
  
  void imageCB(const ImageConstPtr& msg);
  void sonarCB(const ros::MessageEvent<Int16 const>& event);
  void twistCB(const Twist::ConstPtr& msg);

  ros::NodeHandle nh;

  bool snapshot, recording;
  int snap_count, record_count;

  string directory;

  int sonar_data[3];
  double x_vel, z_vel;

  ofstream dataFile;
  VideoWriter* vidFile;

  ros::ServiceServer srv_save_image, srv_start_recording, srv_stop_recording;
  ros::Subscriber image_sub, sonar_1_sub, sonar_2_sub, sonar_3_sub, twist_sub, record_toggle_sub;
};

DataRecorder::DataRecorder(void)
{
  ros::NodeHandle _nh("~");
  srv_save_image = _nh.advertiseService("save_image", &DataRecorder::saveImageCB, this);

  srv_start_recording = _nh.advertiseService("start_recording", &DataRecorder::startRecordingCB, this);
  srv_stop_recording = _nh.advertiseService("stop_recording", &DataRecorder::stopRecordingCB, this);


  image_sub = nh.subscribe("/cv_camera/image_raw", 10, &DataRecorder::imageCB, this);
  sonar_1_sub = nh.subscribe("/arduino/sonar_1", 10, &DataRecorder::sonarCB, this);
  sonar_2_sub = nh.subscribe("/arduino/sonar_2", 10, &DataRecorder::sonarCB, this);
  sonar_3_sub = nh.subscribe("/arduino/sonar_3", 10, &DataRecorder::sonarCB, this);
  twist_sub = nh.subscribe("/cmd_vel", 10, &DataRecorder::twistCB, this);

  nh.param<string>("data_recorder/directory", directory, "/home/ubuntu/");

  snapshot = false;
  recording = false;

  snap_count = 0;
  record_count = 0;

  ros::spin();
}

bool DataRecorder::saveImageCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  snapshot = true;
  return true;
}

bool DataRecorder::startRecordingCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  recording = true;
  string dataFilename = directory + "data_" + to_string(record_count) + ".csv";
  string vidFilename = directory + "vid_" + to_string(record_count) + ".avi";
  dataFile.open(dataFilename.c_str());
  //delete vidFile;
  vidFile =  new VideoWriter();
  vidFile->open(vidFilename,CV_FOURCC('M','J','P','G'), 30, cv::Size(320,240));
  printf("%d\n", vidFile->isOpened());
  record_count++;
  return true;
}

bool DataRecorder::stopRecordingCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  recording = false;
  dataFile.close();
  dataFile.clear();
  return true;
}

void DataRecorder::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
  if (snapshot || recording) {
     Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
     
     if(snapshot) {
       cv::imwrite(directory + "img_" + to_string(snap_count) + ".png", img);
       snap_count++;
     }     

     if(recording) {
       for (int i = 0; i < 3; i++)
         dataFile << sonar_data[i] << ",";
       dataFile << x_vel << "," << z_vel << std::endl;
       *vidFile << img;
     }
  }
  snapshot = false;
}

void DataRecorder::sonarCB(const ros::MessageEvent<Int16 const>& event)
{
  const Int16::ConstPtr& msg = event.getConstMessage();
  const ros::M_string& header = event.getConnectionHeader();
  string topic = header.at("topic");

  sonar_data[topic.at(topic.length()-1) - 49] = msg->data;
}

void DataRecorder::twistCB(const Twist::ConstPtr& msg) {
  x_vel = msg->linear.x;
  z_vel = msg->angular.z;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "data_recorder");
  DataRecorder dr;

  return 0;
}

// vim: ai:ts=2:sw=2:et
