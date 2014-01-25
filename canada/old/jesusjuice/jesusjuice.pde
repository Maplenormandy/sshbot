#include <ros.h>
#include <std_msgs/UInt16.h>

int PWM_PIN = 11;
int DIR_PIN = 29;
uint16 powpow = 30000;


void motorCb(const std_msgs::UInt16& msg)
{
  powpow = msg.data;
}

//ros::NodeHandle nh;
//ros::Subscriber<std_msgs::UInt16> sub("motor", &motorCb );
uint8 dir = HIGH;

void setup()
{
  pinMode(BOARD_LED_PIN, PWM);
  pinMode(PWM_PIN, PWM);
  pinMode(DIR_PIN, OUTPUT);

  //nh.initNode();
  //nh.subscribe(sub);
}

void loop()
{
  powpow += 5000;
  if (powpow > 60000)
  {
    dir = dir == HIGH ? LOW: HIGH;
    powpow = 0;
  }
  pwmWrite(BOARD_LED_PIN, powpow);
  pwmWrite(PWM_PIN, powpow);
  digitalWrite(DIR_PIN, dir);

  //nh.spinOnce();
  delay(500);
}


