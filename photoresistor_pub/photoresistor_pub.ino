/*
   rosserial Publisher Example
   Prints "hello world!"
*/

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32MultiArray.h>

//used for the target teensy 3.6 device
//const int adc_pins[20] = {A9, A8, A7, A6, A5, A4, A3, A2, A1, A0, A22, A21, A20, A19, A18, A17, A16, A15, A14, A13};
//Used when testing on teensy 3.2
const int adc_pins[20] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19};

ros::NodeHandle  nh;

std_msgs::UInt32MultiArray light_msg;
ros::Publisher lights("lights", &light_msg);


void setup()
{
  nh.initNode();
  light_msg.data = (unsigned long*)malloc(sizeof(unsigned long) * 20);
  light_msg.data_length = 20;

  /* Initial Test Vector
    light_msg.data[0] = 69; //Memery
    light_msg.data[1] = 69; //Memery
    light_msg.data[2] = 4294967295; //Maximum
    light_msg.data[3] = 4294967296; //Overflow
  */
  //8 default, 12 reccomened to avoid noise, 16 max in hardware but will introduce noise
  analogReadResolution(16);
  for (int i = 0; i < 20; i++) {
    light_msg.data[i] = 0;
  }

  nh.advertise(lights);
}



void update_msg(std_msgs::UInt32MultiArray& light_msg) {
  /* Initial Test Method
    for (int i = 0; i < 20; i++) {
    light_msg.data[i] += 1;
    }*/
  //For use on actual system  
  for (int i = 0; i < 20; i++) {
    light_msg.data[i] = analogRead(adc_pins[i]);
  }
}

void loop()
{
  lights.publish(&light_msg);

  update_msg(light_msg);
  nh.spinOnce();
  delay(1000);
}
