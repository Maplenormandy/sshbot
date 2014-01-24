#ifndef _ROS_SERVICE_BallDump_h
#define _ROS_SERVICE_BallDump_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace b2b
{

static const char BALLDUMP[] = "b2b/BallDump";

  class BallDumpRequest : public ros::Msg
  {
    public:
      char * color;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_color = strlen( (const char*) this->color);
      memcpy(outbuffer + offset, &length_color, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->color, length_color);
      offset += length_color;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_color;
      memcpy(&length_color, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_color; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_color-1]=0;
      this->color = (char *)(inbuffer + offset-1);
      offset += length_color;
     return offset;
    }

    const char * getType(){ return BALLDUMP; };
    const char * getMD5(){ return "89e44dcab627a2c43a70ae1100695caa"; };

  };

  class BallDumpResponse : public ros::Msg
  {
    public:
      uint16_t dumped;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->dumped >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dumped >> (8 * 1)) & 0xFF;
      offset += sizeof(this->dumped);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->dumped =  ((uint16_t) (*(inbuffer + offset)));
      this->dumped |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->dumped);
     return offset;
    }

    const char * getType(){ return BALLDUMP; };
    const char * getMD5(){ return "bbbc66df16a8c0d0de6993c755ed2a8d"; };

  };

  class BallDump {
    public:
    typedef BallDumpRequest Request;
    typedef BallDumpResponse Response;
  };

}
#endif
