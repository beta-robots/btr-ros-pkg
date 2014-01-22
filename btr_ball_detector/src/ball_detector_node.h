
//std
#include <string>

//external lib
#include "ballDetector.h"

//ros dependencies
#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <btr_ball_detector/detectorParamsConfig.h>

/** \brief wrapping class
 * 
 * Wraps ballDetector object and implement ROS comm's
 * 
 **/
class CballDetectorNode
{
      protected:
            CballDetector detector;//ball_detector library object
            ros::NodeHandle nh;
            image_transport::ImageTransport it;
            image_transport::Subscriber imageSubs;            
            cv_bridge::CvImagePtr cvImgPtrSubs;            
            image_transport::Publisher imagePub;            
            cv_bridge::CvImage cvImgPub;                        
            btr_ball_detector::detectorParamsConfig config;//dynamic reconfigure            
            bool newImageFlag;
            
      public:
            CballDetectorNode();
            ~CballDetectorNode();
            
            bool newImage();
            void process();            
            void publish();
            void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};
