#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"

serial::Serial ser;

void write_callback(const std_msgs::String::ConstPtr& msg) 
{ 
  ROS_INFO_STREAM("Writing to serial port" << msg->data); 
  ser.write(msg->data);   //发送串口数据 
} 

int main(int argc, char **argv){
  ros::init(argc, argv, "serial_example_node");
  ros::NodeHandle nh;

  ros::Subscriber chatter_sub = nh.subscribe("chatter", 1000, write_callback);
  ros::Publisher sensor_pub = nh.advertise<std_msgs::String>("sensor", 1000);
  ser.setPort("/dev/ttyUSB0");
  ser.setBaudrate(115200);
  serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
  ser.setTimeout(to); 
  ser.open(); 
  if(ser.isOpen()){
    ROS_INFO("serial port initialized"); 
  }
  ros::Rate loop_rate(50);
  while(ros::ok()){
    if(ser.available()){
      std_msgs::String result;
      result.data = ser.read(ser.available());
      sensor_pub.publish(result);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
