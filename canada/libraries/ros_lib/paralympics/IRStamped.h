#ifndef _ROS_paralympics_IRStamped_h
#define _ROS_paralympics_IRStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "paralympics/IRArray.h"

namespace paralympics
{

  class IRStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float fwd;
      paralympics::IRArray l;
      paralympics::IRArray r;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_fwd;
      u_fwd.real = this->fwd;
      *(outbuffer + offset + 0) = (u_fwd.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fwd.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fwd.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fwd.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fwd);
      offset += this->l.serialize(outbuffer + offset);
      offset += this->r.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_fwd;
      u_fwd.base = 0;
      u_fwd.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fwd.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fwd.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fwd.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fwd = u_fwd.real;
      offset += sizeof(this->fwd);
      offset += this->l.deserialize(inbuffer + offset);
      offset += this->r.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "paralympics/IRStamped"; };
    const char * getMD5(){ return "3101b5443f0a9585d5a1ed4dfb810211"; };

  };

}
#endif