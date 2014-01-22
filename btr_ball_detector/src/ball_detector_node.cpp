#include "ball_detector_node.h"

CballDetectorNode::CballDetectorNode() : nh(ros::this_node::getName()) , it(this->nh)
{
      detectorConfig detCfg;

      //get user parameters from dynamic reconfigure (yaml entries)
      nh.getParam("gaussian_blur_size", detCfg.gaussian_blur_size); 
      nh.getParam("gaussian_blur_sigma", detCfg.gaussian_blur_sigma); 
      nh.getParam("canny_edge_th", detCfg.canny_edge_th); 
      nh.getParam("hough_accum_resolution", detCfg.hough_accum_resolution);       
      nh.getParam("min_circle_dist", detCfg.min_circle_dist); 
      nh.getParam("hough_accum_th", detCfg.hough_accum_th); 
      nh.getParam("min_radius", detCfg.min_radius);
      nh.getParam("max_radius", detCfg.max_radius); 

      //sets publishers
      imagePub = it.advertise("image_out", 100);
      
      //sets subscribers
      imageSubs = it.subscribe("image_in", 1, &CballDetectorNode::imageCallback, this);
      
      //sets config and prints it
      detector.setParameters(detCfg);
      detector.printConfig();
      
      //resets newImageFlag
      newImageFlag = false;
}

CballDetectorNode::~CballDetectorNode()
{
      
}

bool CballDetectorNode::newImage()
{
      if ( newImageFlag )
      {
            newImageFlag = false;
            return true;
      }
      else
      {
            return false;
      }
}

void CballDetectorNode::process()
{
      if ( cvImgPtrSubs!=NULL )
      {
            //std::cout << "process()!" << std::endl;
            
            //sets input image
            detector.setInputImage(cvImgPtrSubs->image);
            
            //detect circles
            detector.houghDetection();
      }
}

void CballDetectorNode::publish()
{
      //image_raw topic
      cvImgPub.header.seq ++;
      cvImgPub.header.stamp = ros::Time::now();
      cvImgPub.header.frame_id = "detector"; //To do: get frame_id from input image
      cvImgPub.encoding = sensor_msgs::image_encodings::BGR8;
      detector.getOutputImage(cvImgPub.image);
      imagePub.publish(cvImgPub.toImageMsg());
}

void CballDetectorNode::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
      try
      {
            //std::cout << "imageCallback()!" << std::endl;
            this->cvImgPtrSubs = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            //newImageFlag = true;
      }
      catch (cv_bridge::Exception& e)
      {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
      }
      
      //indicates a new image is available
      newImageFlag = true;
      return; 
}
