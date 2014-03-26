#include "point_tracker_node.h"

CpointTrackerNode::CpointTrackerNode() : nh(ros::this_node::getName()) , it(this->nh)
{
      pointTrackerParams params; 
      int auxParam; 
      
      //get user parameters from dynamic reconfigure (yaml entries)
      nh.getParam("verbose_mode", auxParam); 
      params.verbose = (bool)auxParam;
      nh.getParam("min_hessian", params.min_hessian);
      nh.getParam("min_features", auxParam);
      params.min_features = (unsigned int)auxParam;
      nh.getParam("max_correspondence_dist", params.max_correspondence_dist);
      nh.getParam("feature_type", auxParam);
      params.featureType = (featureTypeEnum)auxParam; 
      
      //Fixed params (not set by the user)
      params.old_track_latency = 0;
      params.viewMode = VIEW_POINTS;
      
      //sets config and prints it
      tracker.setParameters(params);
      
      //sets publishers
      imagePub = it.advertise("image_out", 100);
      tracksPub = nh.advertise<btr_point_tracker::pointTracks>("tracks_out", 100);
      
      //sets subscribers
      imageSubs = it.subscribe("image_in", 1, &CpointTrackerNode::imageCallback, this);
      nh.subscribe("cameraInfo_in", 100, &CpointTrackerNode::cameraInfoCallback, this);
            
      //initializes newImageFlag
      newImageFlag = false;
}

CpointTrackerNode::~CpointTrackerNode()
{
      
}

bool CpointTrackerNode::newImage()
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

void CpointTrackerNode::process()
{
      if ( cvImgPtrSubs!=NULL )
      {
            //find point features 
            tracker.findFeatures(cvImgPtrSubs->image,false);
            
            //find point pair correspondences
            tracker.findCorrespondences();
            
            //update individual feature tracks
            tracker.updateTracks();
            
            //switch frame and keypoint buffers
            tracker.switchBuffers();
      }
}

void CpointTrackerNode::publishImage()
{
      //image_raw topic
      cvImgPub.header.seq ++;
      cvImgPub.header.stamp.sec = this->tsec;
      cvImgPub.header.stamp.nsec = this->tnsec;
      cvImgPub.header.frame_id = "detector"; //To do: get frame_id from input image
      cvImgPub.encoding = this->imgEncoding;
      cvImgPub.image = tracker.getDisplayImage();
      imagePub.publish(cvImgPub.toImageMsg());
}

void CpointTrackerNode::publishTracks()
{
      int ii; 
      std::list<CtrackStamped>::iterator iiTrack;
      
      //gets track data from tracker
      std::list<CtrackStamped> & trackList = tracker.getTrackList();
      
      //clears and resize the message
      tracksMsg.tracks.clear();
      tracksMsg.tracks.resize(trackList.size());
      
      //fill header
      tracksMsg.header.seq ++;
      tracksMsg.header.stamp.sec = this->tsec;
      tracksMsg.header.stamp.nsec = this->tnsec;      
      tracksMsg.header.frame_id = "tracker"; //To do: get frame_id from input image
      
      //fill circle data
      ii=0;
      for(iiTrack = trackList.begin(); iiTrack != trackList.end(); iiTrack++ )
      {
            tracksMsg.tracks[ii].x = 0;//iiTrack  getX
            tracksMsg.tracks[ii].y = 0;//iiTrack getY
            tracksMsg.tracks[ii].z = (double)iiTrack->getId();
      }
      
      //publish message
      tracksPub.publish(tracksMsg);
}

void CpointTrackerNode::imageCallback(const sensor_msgs::ImageConstPtr & msg)
{
      try
      {
            this->imgEncoding = msg->encoding;//get image encodings
            this->cvImgPtrSubs = cv_bridge::toCvCopy(msg, msg->encoding);//get image
      }
      catch (cv_bridge::Exception& e)
      {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
      }      
      
      //indicates a new image is available
      this->tsec = msg->header.stamp.sec;
      this->tnsec = msg->header.stamp.nsec;
      this->newImageFlag = true;
      return; 
}

void CpointTrackerNode::cameraInfoCallback(const sensor_msgs::CameraInfo & msg)
{
      // to do ??
}
