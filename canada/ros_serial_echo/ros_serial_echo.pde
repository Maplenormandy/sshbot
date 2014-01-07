/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

void setup()
{
    pinMode(BOARD_LED_PIN, OUTPUT);
    toggleLED();
    
    nh.initNode();
    nh.advertise(chatter);
}

void loop()
{
    toggleLED();
    
    str_msg.data = hello;
    chatter.publish( &str_msg );
    nh.spinOnce();
    delay(100);
}
