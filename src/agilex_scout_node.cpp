#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include "agilex_scout_protocol.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>


class ChassisDriver
{
private:
  uint8_t agilex_can_msg_check_sum(unsigned int id, uint8_t * data, uint8_t len);
  void pack_control_msg(double linear_x, double angular_z);
  void pack_and_publish_odom(double vx, double vy, double vth);
  void read_parse_msg(void);
  void init_can0(void);
  void control_cmd_timer_callback(const ros::TimerEvent& event);
  void twist_callback(const geometry_msgs::Twist::ConstPtr& msg);

public:
  ChassisDriver();
  ~ChassisDriver();

  void run();
private:
  int s, nbytes;
  struct sockaddr_can addr;
  struct ifreq ifr;
  struct can_frame tx_frame;
  struct can_frame rx_frame;
  struct can_filter rfilter[1];

  uint8_t cmd_count;
  ros::Time current_time, last_time, last_twist_time_;
  double x, y, th;
  double vx, vth;
  geometry_msgs::Twist current_twist_;
  ros::Publisher odom_pub;
  tf::TransformBroadcaster odom_broadcaster;
  int control_rate;
  bool publish_tf;
};

ChassisDriver::ChassisDriver():cmd_count(0), x(0), y(0), th(0), vx(0), vth(0){}

ChassisDriver::~ChassisDriver(){
   close(s);
}


uint8_t ChassisDriver::agilex_can_msg_check_sum(unsigned int id, uint8_t * data, uint8_t len)
{
  uint8_t checksum = 0x00;
  checksum = (uint8_t)(id & 0x00ff) + (uint8_t) (id >> 8) + len;
  for (uint8_t i = 0; i < (len - 1); i++) {
    checksum += data[i];
  }
  return checksum;
}

void ChassisDriver::pack_control_msg(double linear_x, double angular_z) {
  int8_t linear_x_percent, angular_z_percent;

  if(linear_x > MAX_LIN_VELOCITY) linear_x_percent = 100; //100%
  else if(linear_x < -MAX_LIN_VELOCITY) linear_x_percent = -100;
  else linear_x_percent = (int8_t)((linear_x*100)/MAX_LIN_VELOCITY);

  if(angular_z > MAX_ANG_VELOCITY) angular_z_percent = 25; //100%
  else if(angular_z < -MAX_ANG_VELOCITY) angular_z_percent = -25;
  else angular_z_percent = (int8_t)((angular_z*100)/PI);

  tx_frame.can_id = CAN_ID_MOTION_CONTROL;
  tx_frame.can_dlc = 8;
  tx_frame.data[0] = CMD_MODE;
  tx_frame.data[1] = NO_CLEAR;
  tx_frame.data[2] = linear_x_percent;
  tx_frame.data[3] = angular_z_percent;
  tx_frame.data[6] = cmd_count++;
  tx_frame.data[7] = agilex_can_msg_check_sum(tx_frame.can_id, tx_frame.data, tx_frame.can_dlc);
}

void ChassisDriver::pack_and_publish_odom(double vx, double vy, double vth)
{
  current_time = ros::Time::now();
  //bug of Agilex can data
  if(vx > 1.5 ) vx = 0;
  if(vth > 0.7853) vth = 0;

  //compute odometry in a typical way given the velocities of the robot
  double dt = (current_time - last_time).toSec();
  double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
  double delta_th = vth * dt;

  x += delta_x;
  y += delta_y;
  th += delta_th;

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  //first, we'll publish the transform over tf
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  //send the transform
  if(publish_tf) odom_broadcaster.sendTransform(odom_trans);

  //next, we'll publish the odometry message over ROS
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";

  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = vth;

    //publish the message
  odom_pub.publish(odom);
}

void ChassisDriver::read_parse_msg(void) {

  unsigned int battery_volt=0;
  int16_t temp;
  int16_t odom_linear_x, odom_angular_z;

  while(1)
  {
    nbytes = read(s, &rx_frame, sizeof(rx_frame));
    if(nbytes > 0)
    {
      if(rx_frame.data[7] != agilex_can_msg_check_sum(rx_frame.can_id, rx_frame.data, rx_frame.can_dlc)) {
        ROS_ERROR("rx frame checksum error!");
        break;
      }
      switch (rx_frame.can_id) {
        case CAN_ID_CHASSIS_STATUS:
        battery_volt = rx_frame.data[2] * 256 + rx_frame.data[3];
        //ROS_INFO("battery_volt: %x", battery_volt);
        break;
        case CAN_ID_MOTION_STATUS:
        temp = rx_frame.data[0] << 8;
        odom_linear_x = temp + rx_frame.data[1];
        temp = rx_frame.data[2] << 8;
        odom_angular_z = temp + rx_frame.data[3];
        vx = (double)odom_linear_x/1000;
        vth = (double)odom_angular_z/1000;
        //ROS_INFO("int odom_linear_x: %d, odom_angular_z: %d", odom_linear_x, odom_angular_z);
        pack_and_publish_odom(vx, 0, vth);
        break;
        default:
        break;
      }
    }
  }
}

void ChassisDriver::init_can0(void) {
  s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  strcpy(ifr.ifr_name, "can0" );
  ioctl(s, SIOCGIFINDEX, &ifr);
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  bind(s, (struct sockaddr *)&addr, sizeof(addr));

  rfilter[0].can_id = 0x00;
  rfilter[0].can_mask = 0;
  setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
}

void ChassisDriver::twist_callback(const geometry_msgs::Twist::ConstPtr& msg){
  last_twist_time_ = ros::Time::now();
  current_twist_ = *msg;
}

void ChassisDriver::control_cmd_timer_callback(const ros::TimerEvent& event) {
  double linear_speed, angular_speed;
  if ((ros::Time::now() - last_twist_time_).toSec()<=1.0){
    linear_speed = current_twist_.linear.x;
    angular_speed = current_twist_.angular.z;
  }else{
    linear_speed = 0;
    angular_speed = 0;
  }
  pack_control_msg(linear_speed, angular_speed);
  nbytes = write(s, &tx_frame, sizeof(tx_frame));
  if(nbytes != sizeof(tx_frame))
  {
      ROS_ERROR("Send Error tx_frame!");
  }
}

void ChassisDriver::run(){
  ros::NodeHandle node;
  ros::NodeHandle private_node("~");

  private_node.param<int>("control_rate", control_rate, 50);  //50hz

  private_node.param<bool>("publish_tf", publish_tf, false);

  init_can0();
  odom_pub = node.advertise<nav_msgs::Odometry>("odom", 10);
  ros::Subscriber cmd_sub = node.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, &ChassisDriver::twist_callback, this);
  ros::Timer control_cmd_timer = node.createTimer(ros::Duration(1.0/control_rate), &ChassisDriver::control_cmd_timer_callback, this);
  boost::thread parse_thread(boost::bind(&ChassisDriver::read_parse_msg, this));
  ros::spin();
  return;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "agilex_scout_node");
  ChassisDriver driver;
  driver.run();
  return 0;
}
