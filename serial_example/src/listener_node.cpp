#define WheelDistance 0.2
#define encoder_sampling_time 0.04
#define speed_ratio 0.0001

#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "serial/serial.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"


/*
void callback(const std_msgs::String::ConstPtr& msg){
  ROS_INFO("listener got [%s]",msg->data.c_str());
}
*/
serial::Serial ser;


int main(int argc, char **argv){
  ros::init(argc, argv, "listener_node");
  ros::NodeHandle nh;

  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Publisher wheelspeed_left_pub = nh.advertise<std_msgs::Float32>("wheelspeed_left", 50);
  ros::Publisher wheelspeed_right_pub = nh.advertise<std_msgs::Float32>("wheelspeed_right", 50);

  tf::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  nav_msgs::Odometry odom;
  std_msgs::Float32 wheelspeed_left;
  std_msgs::Float32 wheelspeed_right;
  geometry_msgs::Quaternion odom_quat;

  int receive_flag = 0;
  float position_x = 0;
  float position_y = 0;
  float oriention = 0;
  float velocity_linear, velocity_angular;
  std_msgs::UInt8MultiArray readbuf;
  readbuf.data.resize(11);
  for(int i = 0; i < 11; i++){
    readbuf.data[i] = 0;
  }

  ser.setPort("/dev/ttyUSB0");
  ser.setBaudrate(115200);
  serial::Timeout to = serial::Timeout::simpleTimeout(1000); 
  ser.setTimeout(to); 
  ser.open(); 
  
  float covariance[36] = {0.01, 0, 0, 0, 0, 0, 
                             0, 0.01, 0, 0, 0, 0, 
                           0, 0, 99999, 0, 0, 0, 
                           0, 0, 0, 99999, 0, 0,
                           0, 0, 0, 0, 99999, 0,
                           0, 0, 0, 0, 0, 0.01};
  for(int i = 0; i < 36; i++){
    odom.pose.covariance[i] = covariance[i];
  }
  ros::Rate loop_rate(20);
  if(ser.isOpen()){
    ROS_INFO("serial initialized!");
  }
  
  double delta_encoder_left, delta_encoder_right, delta_x, delta_y;
  ROS_INFO("listener_node initialized");
  int num = 0;
  while(ros::ok()){
    std_msgs::String m;
    std::stringstream ss_data;
    ss_data << 0;
    m.data = ss_data.str();
    ser.write(m.data);
    //if(receive_flag == 0){
    if(1){
      int p = ser.available();
      if(p){
        if(num < 11){
          ROS_INFO("num: %d", p);
          std_msgs::UInt8MultiArray  serial_data;
          ser.read(serial_data.data, p);
          for(int i = 0; i < p; i++){
            ROS_INFO("serial_data.data[%d], %d", i, serial_data.data[i]);
          } 
          //if(p < 12){
          //int num_ = num;
          for(int i = 0; (((i +num) < 11 ) && (i < p)); ++i){
            //ROS_INFO("got msg: %d", serial_data.data[i]);
            //int i_ = (num + i) % 11;
            readbuf.data[num + i] = serial_data.data[i]; 
            //ROS_INFO("readbuf[%d]: %d", num + i, readbuf.data[num + i]);
            //}
          }
          num += p;
       // num += p;
        }
        if(num >= 11){
          num = num % 11;
          for(int i = 0; i < 11; i++){
           ROS_INFO("readbuf[%d]: %d", i, readbuf.data[i]); 
          }
        } 
      }
      


      if((readbuf.data[0] == 0xff) && (readbuf.data[1] == 0xff)){
        uint8_t check_sum = 0;
        for(int i = 0; i < 10; i++){
          check_sum += readbuf.data[i];
        }
        if(readbuf.data[10] == check_sum){
          ROS_INFO("GOT RIGHT MSG");
          delta_encoder_left = (readbuf.data[2] > 0? 1:-1) * ((readbuf.data[3] << 16) + (readbuf.data[4] << 8) + readbuf.data[5]);
          delta_encoder_right = (readbuf.data[6] > 0? 1:-1) * ((readbuf.data[7] << 16) + (readbuf.data[8] << 8) + readbuf.data[9]);
          float delta_d_left, delta_d_right;
          delta_d_left = delta_encoder_left * speed_ratio;
          delta_d_right = delta_encoder_right * speed_ratio;
          float delta_d = (delta_d_left + delta_d_right) * 0.5;
          float delta_theta = (delta_d_right - delta_d_left) / WheelDistance;
          delta_x = delta_d * cos(oriention + delta_theta * 0.5);
          delta_y = delta_d * sin(oriention + delta_theta * 0.5);
          position_x += delta_x;
          position_y += delta_y; 
          oriention += delta_theta;
          velocity_linear = delta_d / encoder_sampling_time;
          velocity_angular = delta_theta / encoder_sampling_time;
          receive_flag = 1;
          for(int i = 0; i < 11; i++){
            readbuf.data[i] = 0;
          }
        }
      }
    }
    if(receive_flag == 1){
      odom_quat = tf::createQuaternionMsgFromYaw(oriention);
      odom_trans.header.stamp = ros::Time::now();
      odom_trans.header.frame_id = "odom";
      odom_trans.child_frame_id = "base_link";
      odom_trans.transform.translation.x = position_x;
      odom_trans.transform.translation.y = position_y;
      odom_trans.transform.translation.z = 0.0;  
      odom_trans.transform.rotation = odom_quat;
 
      odom_broadcaster.sendTransform(odom_trans);

      odom.header.stamp = ros::Time::now();
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_link";
      odom.pose.pose.position.x = position_x;
      odom.pose.pose.position.y = position_y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation = odom_quat;  
      odom.twist.twist.linear.x = velocity_linear;
      odom.twist.twist.angular.z = velocity_angular;
    
      odom_pub.publish(odom);

      wheelspeed_left.data = delta_encoder_left * speed_ratio / encoder_sampling_time;
      wheelspeed_right.data = delta_encoder_right * speed_ratio / encoder_sampling_time;
      wheelspeed_left_pub.publish(wheelspeed_left);
      wheelspeed_right_pub.publish(wheelspeed_right);

    }
  }





  ros::spin();
  loop_rate.sleep();
  return 0;


}
