#ifndef _ROS_visualization_msgs_InteractiveMarkerUpdate_h
#define _ROS_visualization_msgs_InteractiveMarkerUpdate_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "visualization_msgs/InteractiveMarker.h"
#include "visualization_msgs/InteractiveMarkerPose.h"

namespace visualization_msgs
{

  class InteractiveMarkerUpdate : public ros::Msg
  {
    public:
      char * server_id;
      uint64_t seq_num;
      uint8_t type;
      uint8_t markers_length;
      visualization_msgs::InteractiveMarker st_markers;
      visualization_msgs::InteractiveMarker * markers;
      uint8_t poses_length;
      visualization_msgs::InteractiveMarkerPose st_poses;
      visualization_msgs::InteractiveMarkerPose * poses;
      uint8_t erases_length;
      char* st_erases;
      char* * erases;
      enum { KEEP_ALIVE =  0 };
      enum { UPDATE =  1 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_server_id = strlen( (const char*) this->server_id);
      memcpy(outbuffer + offset, &length_server_id, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->server_id, length_server_id);
      offset += length_server_id;
      *(outbuffer + offset + 0) = (this->seq_num >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->seq_num >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->seq_num >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->seq_num >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->seq_num >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->seq_num >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->seq_num >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->seq_num >> (8 * 7)) & 0xFF;
      offset += sizeof(this->seq_num);
      *(outbuffer + offset + 0) = (this->type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->type);
      *(outbuffer + offset++) = markers_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < markers_length; i++){
      offset += this->markers[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = poses_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < poses_length; i++){
      offset += this->poses[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = erases_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < erases_length; i++){
      uint32_t length_erasesi = strlen( (const char*) this->erases[i]);
      memcpy(outbuffer + offset, &length_erasesi, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->erases[i], length_erasesi);
      offset += length_erasesi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_server_id;
      memcpy(&length_server_id, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_server_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_server_id-1]=0;
      this->server_id = (char *)(inbuffer + offset-1);
      offset += length_server_id;
      this->seq_num =  ((uint64_t) (*(inbuffer + offset)));
      this->seq_num |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->seq_num |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->seq_num |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->seq_num |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->seq_num |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->seq_num |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->seq_num |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->seq_num);
      this->type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->type);
      uint8_t markers_lengthT = *(inbuffer + offset++);
      if(markers_lengthT > markers_length)
        this->markers = (visualization_msgs::InteractiveMarker*)realloc(this->markers, markers_lengthT * sizeof(visualization_msgs::InteractiveMarker));
      offset += 3;
      markers_length = markers_lengthT;
      for( uint8_t i = 0; i < markers_length; i++){
      offset += this->st_markers.deserialize(inbuffer + offset);
        memcpy( &(this->markers[i]), &(this->st_markers), sizeof(visualization_msgs::InteractiveMarker));
      }
      uint8_t poses_lengthT = *(inbuffer + offset++);
      if(poses_lengthT > poses_length)
        this->poses = (visualization_msgs::InteractiveMarkerPose*)realloc(this->poses, poses_lengthT * sizeof(visualization_msgs::InteractiveMarkerPose));
      offset += 3;
      poses_length = poses_lengthT;
      for( uint8_t i = 0; i < poses_length; i++){
      offset += this->st_poses.deserialize(inbuffer + offset);
        memcpy( &(this->poses[i]), &(this->st_poses), sizeof(visualization_msgs::InteractiveMarkerPose));
      }
      uint8_t erases_lengthT = *(inbuffer + offset++);
      if(erases_lengthT > erases_length)
        this->erases = (char**)realloc(this->erases, erases_lengthT * sizeof(char*));
      offset += 3;
      erases_length = erases_lengthT;
      for( uint8_t i = 0; i < erases_length; i++){
      uint32_t length_st_erases;
      memcpy(&length_st_erases, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_erases; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_erases-1]=0;
      this->st_erases = (char *)(inbuffer + offset-1);
      offset += length_st_erases;
        memcpy( &(this->erases[i]), &(this->st_erases), sizeof(char*));
      }
     return offset;
    }

    const char * getType(){ return "visualization_msgs/InteractiveMarkerUpdate"; };
    const char * getMD5(){ return "83e22f99d3b31fde725e0a338200e036"; };

  };

}
#endif