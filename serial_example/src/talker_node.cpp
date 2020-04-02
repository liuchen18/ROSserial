#define WheelDistance 0.2
#define cmd_vel_linear_max 100000
#define cmd_vel_angular_max 100000
#define encoder_sampling_time 0.04
#define speed_ratio 0.0001

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "geometry_msgs/Twist.h"
#include <sstream>
#include "serial/serial.h"

serial::Serial ser;

int send_update_flag = 0;
uint8_t writebuf[11];
uint8_t stopbuf[11];


void Callback(const geometry_msgs::Twist& cmd_vel){
  for(int i = 0; i < 11; i++){
    writebuf[i] = 0x00;
  }
  float angular_temp;
  float linear_temp;
  linear_temp = 400 * (cmd_vel.linear.x);
  angular_temp = 4000 * (cmd_vel.angular.z);

  float linear_max_limit = cmd_vel_linear_max;
  float angular_max_limit = cmd_vel_angular_max;
  if(linear_temp > linear_max_limit){
    linear_temp = linear_max_limit;
  }
  if(linear_temp < (-1 * linear_max_limit)){
    linear_temp = -1 * linear_max_limit;
  }
  if(angular_temp > angular_max_limit){
    angular_temp = angular_max_limit;
  }
  if(angular_temp < (-1 * angular_max_limit)){
    angular_temp = -1 * angular_max_limit;
  }
  
  int delta_encoder_left_temp = (linear_temp - 0.5 * (WheelDistance * angular_temp));// * encoder_sampling_time / speed_ratio;
  int delta_encoder_right_temp = (linear_temp + 0.5 * (WheelDistance * angular_temp));// * encoder_sampling_time / speed_ratio;

  while(send_update_flag != 0);
  writebuf[0]=writebuf[1]=0xff;
  if(delta_encoder_left_temp >= 0){
    writebuf[2] = 0x01;
  }
  else{
    writebuf[2] = 0x00;
  }
  writebuf[3] = abs(delta_encoder_left_temp)>>16;
  writebuf[4] = (abs(delta_encoder_left_temp)>>8)&0xff;
  writebuf[5] = abs(delta_encoder_left_temp)&0xff;
  if(delta_encoder_right_temp >= 0){
    writebuf[6] = 0x01;
  }
  else{
    writebuf[6] = 0x00;
  }
  writebuf[7] = abs(delta_encoder_right_temp)>>16;
  writebuf[8] = (abs(delta_encoder_right_temp)>>8)&0xff;
  writebuf[9] = abs(delta_encoder_right_temp)&0xff;
  int temp = 0;
  for(int i = 0; i <10; i++){
    temp += writebuf[i];
  }
  writebuf[10] = temp & 0xff;
  send_update_flag = 1;

}


int main(int argc, char **argv){
  ros::init(argc,argv,"talker_node");
  ros::NodeHandle nh;
  
  ros::Subscriber cmdvel_sub = nh.subscribe("/turtle1/cmd_vel", 1000, Callback);
  //ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("chatter", 10);
  ros::Rate loop_rate(10);
  
  ser.setPort("/dev/ttyUSB0");
  ser.setBaudrate(115200);
  serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
  ser.setTimeout(to); 
  ser.open(); 
  if(ser.isOpen()){
    ROS_INFO("serial port initialized"); 
  }

  stopbuf[0]=0xff;
  stopbuf[1]=0xff;
  for(int i = 2; i < 10; i++){
    stopbuf[i] = 0x00;
  }
  stopbuf[10]=0xfe;
  int count = 0;
  double send_time;
  while(ros::ok()){
    std_msgs::String msg;
    if(send_update_flag == 1){
      ser.write(writebuf, 11);
      ROS_INFO_STREAM("Writing to serial port");
      send_update_flag = 0;
      send_time = ros::Time::now().toSec();
      ROS_INFO("send_time: %f", send_time);
    }
    if(ros::Time::now().toSec() - send_time > 1.0){
      ser.write(stopbuf, 11);
      //ROS_INFO("stop!!!!");
    }
    ros::spinOnce(); 
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
