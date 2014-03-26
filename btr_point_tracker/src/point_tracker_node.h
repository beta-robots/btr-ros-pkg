
//std
#include <string>
#include <list>

//external lib
#include "pointTracker.h"

//ros dependencies
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "sensor_msgs/CameraInfo.h"
#include <sensor_msgs/image_encodings.h>
#include <btr_point_tracker/pointTracks.h> //message
#include <btr_point_tracker/trackerParamsConfig.h> //configs

/** \brief wrapping class
 * 
 * Wraps pointTracker object and implement ROS comm's
 * 
 **/
class CpointTrackerNode
{
      protected:
            
            //ball_detector library object
            CpointTracker tracker;
            
            //ros node handle
            ros::NodeHandle nh;
            
            //image publisher/subscriber
            image_transport::ImageTransport it;
            image_transport::Publisher imagePub;            
            cv_bridge::CvImage cvImgPub;
            image_transport::Subscriber imageSubs;            
            cv_bridge::CvImagePtr cvImgPtrSubs;            

            //circle set publisher
            btr_point_tracker::pointTracks tracksMsg;            
            ros::Publisher tracksPub;
            
            //camera info subscriber
            sensor_msgs::CameraInfo cameraInfoMsg;
            ros::Subscriber cameraInfoSubs;

            //dynamic reconfigure 
            btr_point_tracker::trackerParamsConfig config; 
            
            //flag indicating a new image has been received
            bool newImageFlag;
                        
            //img encoding id
            std::string imgEncoding;            
            
            //image time stamp
            unsigned int tsec;
            unsigned int tnsec;            
            
      public:
            CpointTrackerNode();
            ~CpointTrackerNode();
            
            bool newImage();
            void process();            
            void publishImage();
            void publishTracks();
            void imageCallback(const sensor_msgs::ImageConstPtr & msg);
            void cameraInfoCallback(const sensor_msgs::CameraInfo & msg);
};
