#include <ros/ros.h>
#include <iostream>
#include "std_msgs/String.h"
#include "arm_lib/cmd_message.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cmd_message_publisher");

  ros::NodeHandle n;
  ros::Publisher p = n.advertise<arm_lib::cmd_message>("command_chatter", 1000);

  arm_lib::cmd_message msg;

   std::string command;

   while (ros::ok())
  {
    std::cout << "Release or Catch command input";
    std::getline(std::cin, command);


    msg.command_name = command;
    ROS_INFO("%s", msg.command_name);
    p.publish(msg);

    ros::spinOnce();
  }

  return 0;
}