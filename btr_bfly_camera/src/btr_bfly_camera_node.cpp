
//external lib
#include "bflyCamera.h"

//opencv
// #include "opencv2/core/core.hpp"
// #include "opencv2/calib3d/calib3d.hpp"
// #include "opencv2/highgui/highgui.hpp"

//ros dependencies
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <btr_bfly_camera/bflyParamsConfig.h>

//std
#include <string>

//fill CameraInfo message from an opencv xml calibration file
int fillCameraInfoMessage(const std::string & fileName, sensor_msgs::CameraInfo &msg)
{
      int ii;
      cv::Mat KK(3,3,CV_64FC1);
      cv::Mat DD(5,1,CV_64FC1);
      cv::FileStorage fs; 
      
      //open file
      fs.open(fileName, cv::FileStorage::READ);
      if ( !fs.isOpened() )
      {
            std::cout << "WARNING: Camera Calibration File " << fileName << " Not Found." << std::endl;
            std:: cout << "WARNING: camera_info topic will not provide right data" << std::endl;
            return BFLY_ERROR;
      }

      //fill static part of the message
      msg.header.frame_id = "bfly_camera";
      msg.binning_x = 0;
      msg.binning_x = 0;
      msg.roi.width = 0;
      msg.roi.height = 0;
      for (ii=0; ii<9; ii++) msg.R[ii] = 0;
      msg.width = (int)fs["image_Width"];
      msg.height = (int)fs["image_Height"];
      fs["Camera_Matrix"] >> KK;
      for (ii=0; ii<9; ii++) msg.K[ii] = KK.at<double>(ii);
      msg.distortion_model = "plumb_bob";
      fs["Distortion_Coefficients"] >> DD;
      for (ii=0; ii<5; ii++) msg.D.push_back(DD.at<double>(ii));
      
      //close file and return
      fs.release();
      return BFLY_SUCCESS;
}

//node main
int main(int argc, char **argv)
{
      //ros core
      ros::init(argc, argv, "btr_bfly_camera_node");
      ros::NodeHandle nh(ros::this_node::getName());
      ros::Rate loop_rate(50);//set node loop rate
      btr_bfly_camera::bflyParamsConfig config;
      
      //image publisher
      image_transport::ImageTransport it(nh);
      image_transport::Publisher imagePub;
      cv_bridge::CvImage cv_img;
      
      //camera info publisher
      ros::Publisher cameraInfoPub;
      sensor_msgs::CameraInfo cameraInfoMsg;
      
      //Camera object from bfly_camera library
      CbflyCamera camera;
      
      //other variables
      unsigned int cameraStatus;
      unsigned int ii;
      int userParamInt;
      bfly_videoMode vMode;
      bfly_pixelFormat pxFormat;
      std::string calibrationFileName;
      
      //advertise publishers
      imagePub = it.advertise("image_raw", 100);
      cameraInfoPub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 100);
      
      //Check if camera has been initialized correctly
      if( !camera.isInitOk() )
      {
            ROS_INFO("Error on Camera init. Exit node.");
            return -1;
      }
      
      //get user parameters from dynamic reconfigure (yaml entries)
      nh.getParam("videoMode", userParamInt); 
      vMode = (bfly_videoMode)userParamInt;
      nh.getParam("pixelFormat", userParamInt); 
      pxFormat = (bfly_pixelFormat)userParamInt;
      nh.getParam("calibrationFile", calibrationFileName); 
      
      //print user parameters
      std::cout << "User Config Params" << std::endl;
      std::cout << "    Video Mode: " << vMode << std::endl; 
      std::cout << "    Pixel Format: " << pxFormat << std::endl;
      std::cout << "    Calibration File: " << calibrationFileName << std::endl;
            
      //connect to camera
      camera.open();
      
      //configure image settings
      camera.configure(vMode, pxFormat);
      
      //print camera information
      camera.printCameraInfo();
      
      //fill the static part of the CameraInfo message      
      fillCameraInfoMessage(calibrationFileName, cameraInfoMsg);
      
      //start image acquisition
      cameraStatus = camera.startAcquisition();
            
      //acquisition loop
      while ( (ros::ok()) && (cameraStatus == BFLY_SUCCESS) )
      {
            //get image
            cameraStatus = camera.getCurrentImage(cv_img.image);
            
            //fill image header and publish image and info
            if ( cameraStatus == BFLY_SUCCESS )
            {
                  //image_raw topic
                  cv_img.header.seq ++;
                  cv_img.header.stamp = ros::Time::now();
                  cv_img.header.frame_id = "bfly_camera";
                  if ( pxFormat == RGB8 ) cv_img.encoding = sensor_msgs::image_encodings::RGB8;
                  if ( pxFormat == MONO8 ) cv_img.encoding = sensor_msgs::image_encodings::MONO8;
                  imagePub.publish(cv_img.toImageMsg());
                  
                  //camera_info topic
                  cameraInfoMsg.header.seq ++;
                  cameraInfoMsg.header.stamp = cv_img.header.stamp; 
                  cameraInfoPub.publish(cameraInfoMsg);
            }
            
            //execute pending callbacks
            ros::spinOnce();

            //sleep up to rate
            loop_rate.sleep();
      }
      
      //Advertise in case loop exit is due to cameraStatus
      if (cameraStatus == BFLY_ERROR) std::cout << "Exit node due to BFLY_ERROR return by CbflyCamera::getCurrentImage()" << std::endl;
      
      //stop image acquisition
      camera.stopAcquisition();
      
      //disconnect from camera
      camera.close();
 
      //return
      return 0;
}