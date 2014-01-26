#ifndef _ROS_SERVICE_Locator_h
#define _ROS_SERVICE_Locator_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Header.h"

namespace nav
{

static const char LOCATOR[] = "nav/Locator";

  class LocatorRequest : public ros::Msg
  {
    public:

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return LOCATOR; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class LocatorResponse : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::Pose reactor1;
      geometry_msgs::Pose reactor2;
      geometry_msgs::Pose reactor3;
      geometry_msgs::Pose silo;
      geometry_msgs::Pose opponent;

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->reactor1.serialize(outbuffer + offset);
      offset += this->reactor2.serialize(outbuffer + offset);
      offset += this->reactor3.serialize(outbuffer + offset);
      offset += this->silo.serialize(outbuffer + offset);
      offset += this->opponent.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->reactor1.deserialize(inbuffer + offset);
      offset += this->reactor2.deserialize(inbuffer + offset);
      offset += this->reactor3.deserialize(inbuffer + offset);
      offset += this->silo.deserialize(inbuffer + offset);
      offset += this->opponent.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return LOCATOR; };
    const char * getMD5(){ return "e0a9a81d54d904efc81fc1efdf42a1bc"; };

  };

  class Locator {
    public:
    typedef LocatorRequest Request;
    typedef LocatorResponse Response;
  };

}
#endif
