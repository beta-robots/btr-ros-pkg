# This represents the set of tracks currently active

#timestamp and frame id of the image frame
Header header

#set of tracks
#      (tracks[i].x, tracks[i].y) is the current track point in image coordinates
#      tracks[i].z is the track id
geometry_msgs/Point[] tracks
