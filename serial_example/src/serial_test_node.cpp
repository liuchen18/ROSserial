#define WheelDistance 0.21
#define encoder_sampling_time 0.04
#define speed_ratio 0.0001
#define cmd_vel_linear_max 100000
#define cmd_vel_angular_max 100000

#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Quaternion.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include <sstream>


serial::Serial ser;
int send_update_flag = 0;
uint8_t writebuf[11];
uint8_t stopbuf[11];


void Callback(const geometry_msgs::Twist& cmd_vel){
  ROS_INFO("GOT CMD_VEL MSG");
  for(int i = 0; i < 11; i++){
    writebuf[i] = 0x00;
  }
  float angular_temp;
  float linear_temp;
  linear_temp = 600 * (cmd_vel.linear.x);
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
  
  int delta_encoder_left_temp = (linear_temp + 0.5 * (WheelDistance * angular_temp));// * encoder_sampling_time / speed_ratio;
  int delta_encoder_right_temp = (linear_temp - 0.5 * (WheelDistance * angular_temp));// * encoder_sampling_time / speed_ratio;

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
  ros::init(argc, argv, "serial_node");
  ros::NodeHandle nh;

  ros::Subscriber cmdvel_sub = nh.subscribe("/turtle1/cmd_vel", 1000, Callback);
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
 
  stopbuf[0]=0xff;
  stopbuf[1]=0xff;
  for(int i = 2; i < 10; i++){
    stopbuf[i] = 0x00;
  }
  stopbuf[10]=0xfe;

  
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
  ROS_INFO("serial_node initialized");
  int empty_flag = 0;
  double send_time;
  int num = 0, bigNum = 0;    
  int flag=0;//0--inputing,1--findingHead
  int p = 0;
  int continue_flag; //0--last msg is over, 1-- this msg should be added to last msg
  std_msgs::UInt8MultiArray  s_data;
  s_data.data.resize(100);
  while(ros::ok()){
    if(send_update_flag == 1){
      ser.write(writebuf, 11);
      ROS_INFO_STREAM("Writing to serial port");
      send_update_flag = 0;
      send_time = ros::Time::now().toSec();
      ROS_INFO("send_time: %f", send_time);
    }
    if(ros::Time::now().toSec() - send_time > 1.0){
      ser.write(stopbuf, 11);
    }
      

      std_msgs::UInt8MultiArray  serial_data;
        p = ser.available();
        //ROS_INFO("P: %d",p);
        ser.read(serial_data.data, p);
      if(p!=0){
        for(int i = 0; i < p; i++){
          s_data.data[bigNum+i] = serial_data.data[i];
        }
        int n = bigNum;
        while (bigNum<(n+p)&&num<11){
           readbuf.data[num++]=s_data.data[bigNum];
           if(continue_flag == 0){
             if (flag==0){
                if (s_data.data[bigNum]==255){
                   flag=1;
                }
             }
             else{//flag==1
                if (s_data.data[bigNum]==255){
                   if(num!=2){
                     readbuf.data[0]=255;
                     readbuf.data[1]=255;
                     num=2;
                     continue_flag = 1;
	           }
                   flag=0;
                }
             }
           }
           bigNum++;
           //ROS_INFO("bigNum: %d", bigNum);
        }
      }

    
    if(num >= 11){
      for(int i = 0; i < 11; i++){
        ROS_INFO("readbuf[%d]:%d",i, readbuf.data[i]);
      }
      //ROS_INFO("M >= 11");
      if((readbuf.data[0] == 0xff) && (readbuf.data[1] == 0xff)){
        uint8_t check_sum = 0;
        for(int i = 0; i < 10; i++){
          check_sum += readbuf.data[i];
        }
        

        if(readbuf.data[10] == check_sum){
          ROS_INFO("GOT RIGHT MSG");
          delta_encoder_right = (readbuf.data[2] > 0? 1:-1) * ((readbuf.data[3] << 16) + (readbuf.data[4] << 8) + readbuf.data[5]);
          delta_encoder_left = (readbuf.data[6] > 0? 1:-1) * ((readbuf.data[7] << 16) + (readbuf.data[8] << 8) + readbuf.data[9]);
          ROS_INFO("delta_encoder_right: %f,delta_encoder_left: %f", delta_encoder_right, delta_encoder_left);
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
          //ROS_INFO("position X: %f,position Y: %f, oriention : %f", position_x, position_y, oriention);
          velocity_linear = delta_d / encoder_sampling_time;
          velocity_angular = delta_theta / encoder_sampling_time;
          receive_flag = 1;
          for(int i = 0; i < 11; i++){
            readbuf.data[i] = 0;
          }
        }
      }
      num = 0;
      bigNum = 0;
      continue_flag = 0;
      for(int i = 0; i < 100; i++){
        s_data.data[i] = 0;
      }
      //ROS_INFO("num = %d", num);
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
      
      //ROS_INFO("once");

    }
    ros::spinOnce();
  }
  
  loop_rate.sleep();
  return 0;


}
