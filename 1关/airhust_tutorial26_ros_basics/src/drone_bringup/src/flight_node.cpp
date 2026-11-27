// 第一关：以固定频率发布设定点，起飞并在障碍物前下降。
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>

double distance = 2.0;

void obstacle_cb(const std_msgs::Float32::ConstPtr& msg) {
  distance = msg->data;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "flight_node");
  ros::NodeHandle nh;

  // 飞控连接有500ms的心跳包（最大间隔不超过0.5s，可以比这快）。
  ros::Rate rate(10.0);

  ros::Publisher g_setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>("/drone/setpoint", 10);

  ros::Subscriber obstacle_sub =
      nh.subscribe("/drone/sensor/obstacle_distance", 10, obstacle_cb);

  geometry_msgs::PoseStamped setpoint;
  setpoint.header.frame_id = "map";
  setpoint.pose.position.x = 0.0;
  setpoint.pose.position.y = 0.0;
  setpoint.pose.position.z = 1.5;



  ROS_INFO("flight_node ready");
  while (ros::ok()) {

    if (distance < 0.5) {
      setpoint.pose.position.z = 0;
    }

    setpoint.header.stamp = ros::Time::now();
    g_setpoint_pub.publish(setpoint);
    ros::spinOnce();
    rate.sleep();
  }

  ROS_INFO("flight_node stopped");
  return 0;
}
