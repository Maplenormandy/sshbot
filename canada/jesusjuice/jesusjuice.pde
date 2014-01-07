#include <ros.h>
#include <std_msgs/UInt16.h>

int PWM_PIN = 2;
int DIR_PIN = 3;
uint16 powpow = 30000;


void motorCb(const std_msgs::UInt16& msg)
{
  powpow = msg.data;
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::UInt16> sub("motor", &motorCb );

void setup()
{
  pinMode(BOARD_LED_PIN, PWM);
  pinMode(PWM_PIN, PWM);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);

  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  pwmWrite(BOARD_LED_PIN, powpow);
  pwmWrite(PWM_PIN, powpow);

  nh.spinOnce();
  delay(16);
}


