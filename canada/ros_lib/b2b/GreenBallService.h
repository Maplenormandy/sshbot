#ifndef _ROS_SERVICE_GreenBallService_h
#define _ROS_SERVICE_GreenBallService_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace b2b
{

static const char GREENBALLSERVICE[] = "b2b/GreenBallService";

  class GreenBallServiceRequest : public ros::Msg
  {
    public:
      uint16_t requested;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->requested >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->requested >> (8 * 1)) & 0xFF;
      offset += sizeof(this->requested);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->requested =  ((uint16_t) (*(inbuffer + offset)));
      this->requested |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->requested);
     return offset;
    }

    const char * getType(){ return GREENBALLSERVICE; };
    const char * getMD5(){ return "58b12a3db60ab8362eee2b9ae934054f"; };

  };

  class GreenBallServiceResponse : public ros::Msg
  {
    public:
      uint16_t tray;
      uint16_t queued;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->tray >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->tray >> (8 * 1)) & 0xFF;
      offset += sizeof(this->tray);
      *(outbuffer + offset + 0) = (this->queued >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->queued >> (8 * 1)) & 0xFF;
      offset += sizeof(this->queued);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->tray =  ((uint16_t) (*(inbuffer + offset)));
      this->tray |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->tray);
      this->queued =  ((uint16_t) (*(inbuffer + offset)));
      this->queued |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->queued);
     return offset;
    }

    const char * getType(){ return GREENBALLSERVICE; };
    const char * getMD5(){ return "750b606410c9bb05fc14e894e6d5e709"; };

  };

  class GreenBallService {
    public:
    typedef GreenBallServiceRequest Request;
    typedef GreenBallServiceResponse Response;
  };

}
#endif
