#ifndef _ROS_b2b_IRStamped_h
#define _ROS_b2b_IRStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "b2b/IRArray.h"

namespace b2b
{

  class IRStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float fwd_l;
      float fwd_r;
      b2b::IRArray l;
      b2b::IRArray r;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_fwd_l;
      u_fwd_l.real = this->fwd_l;
      *(outbuffer + offset + 0) = (u_fwd_l.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fwd_l.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fwd_l.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fwd_l.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fwd_l);
      union {
        float real;
        uint32_t base;
      } u_fwd_r;
      u_fwd_r.real = this->fwd_r;
      *(outbuffer + offset + 0) = (u_fwd_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_fwd_r.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_fwd_r.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_fwd_r.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->fwd_r);
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
      } u_fwd_l;
      u_fwd_l.base = 0;
      u_fwd_l.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fwd_l.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fwd_l.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fwd_l.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fwd_l = u_fwd_l.real;
      offset += sizeof(this->fwd_l);
      union {
        float real;
        uint32_t base;
      } u_fwd_r;
      u_fwd_r.base = 0;
      u_fwd_r.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_fwd_r.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_fwd_r.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_fwd_r.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->fwd_r = u_fwd_r.real;
      offset += sizeof(this->fwd_r);
      offset += this->l.deserialize(inbuffer + offset);
      offset += this->r.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "b2b/IRStamped"; };
    const char * getMD5(){ return "7d7ea158c6b4a318e1ad2024871fc8ce"; };

  };

}
#endif