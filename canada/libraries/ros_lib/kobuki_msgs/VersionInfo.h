#ifndef _ROS_kobuki_msgs_VersionInfo_h
#define _ROS_kobuki_msgs_VersionInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace kobuki_msgs
{

  class VersionInfo : public ros::Msg
  {
    public:
      char * hardware;
      char * firmware;
      char * software;
      uint8_t udid_length;
      uint32_t st_udid;
      uint32_t * udid;
      uint64_t features;
      enum { SMOOTH_MOVE_START =  0000000000000001 };
      enum { GYROSCOPE_3D_DATA =  0000000000000002 };

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_hardware = strlen( (const char*) this->hardware);
      memcpy(outbuffer + offset, &length_hardware, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->hardware, length_hardware);
      offset += length_hardware;
      uint32_t length_firmware = strlen( (const char*) this->firmware);
      memcpy(outbuffer + offset, &length_firmware, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->firmware, length_firmware);
      offset += length_firmware;
      uint32_t length_software = strlen( (const char*) this->software);
      memcpy(outbuffer + offset, &length_software, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->software, length_software);
      offset += length_software;
      *(outbuffer + offset++) = udid_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < udid_length; i++){
      *(outbuffer + offset + 0) = (this->udid[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->udid[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->udid[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->udid[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->udid[i]);
      }
      *(outbuffer + offset + 0) = (this->features >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->features >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->features >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->features >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->features >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->features >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->features >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->features >> (8 * 7)) & 0xFF;
      offset += sizeof(this->features);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_hardware;
      memcpy(&length_hardware, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_hardware; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_hardware-1]=0;
      this->hardware = (char *)(inbuffer + offset-1);
      offset += length_hardware;
      uint32_t length_firmware;
      memcpy(&length_firmware, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_firmware; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_firmware-1]=0;
      this->firmware = (char *)(inbuffer + offset-1);
      offset += length_firmware;
      uint32_t length_software;
      memcpy(&length_software, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_software; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_software-1]=0;
      this->software = (char *)(inbuffer + offset-1);
      offset += length_software;
      uint8_t udid_lengthT = *(inbuffer + offset++);
      if(udid_lengthT > udid_length)
        this->udid = (uint32_t*)realloc(this->udid, udid_lengthT * sizeof(uint32_t));
      offset += 3;
      udid_length = udid_lengthT;
      for( uint8_t i = 0; i < udid_length; i++){
      this->st_udid =  ((uint32_t) (*(inbuffer + offset)));
      this->st_udid |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_udid |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_udid |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_udid);
        memcpy( &(this->udid[i]), &(this->st_udid), sizeof(uint32_t));
      }
      this->features =  ((uint64_t) (*(inbuffer + offset)));
      this->features |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->features |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->features |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->features |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->features |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->features |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->features |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->features);
     return offset;
    }

    const char * getType(){ return "kobuki_msgs/VersionInfo"; };
    const char * getMD5(){ return "3dc0ce6edd49be1fc86730f4a4a21861"; };

  };

}
#endif