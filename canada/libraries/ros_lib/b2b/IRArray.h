#ifndef _ROS_b2b_IRArray_h
#define _ROS_b2b_IRArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace b2b
{

  class IRArray : public ros::Msg
  {
    public:
      float fwd;
      float mid;
      float bak;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
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
      union {
        float real;
        uint32_t base;
      } u_mid;
      u_mid.real = this->mid;
      *(outbuffer + offset + 0) = (u_mid.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mid.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_mid.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_mid.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mid);
      union {
        float real;
        uint32_t base;
      } u_bak;
      u_bak.real = this->bak;
      *(outbuffer + offset + 0) = (u_bak.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_bak.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_bak.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_bak.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->bak);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
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
      union {
        float real;
        uint32_t base;
      } u_mid;
      u_mid.base = 0;
      u_mid.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mid.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_mid.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_mid.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->mid = u_mid.real;
      offset += sizeof(this->mid);
      union {
        float real;
        uint32_t base;
      } u_bak;
      u_bak.base = 0;
      u_bak.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_bak.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_bak.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_bak.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->bak = u_bak.real;
      offset += sizeof(this->bak);
     return offset;
    }

    const char * getType(){ return "b2b/IRArray"; };
    const char * getMD5(){ return "ebda03d9708d4be22b326ce455df8102"; };

  };

}
#endif