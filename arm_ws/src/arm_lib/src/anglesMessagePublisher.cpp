#include <ros/ros.h>
#include <iostream>
#include "std_msgs/String.h"
#include "arm_lib/angles_message.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "angles_message_publisher");

  ros::NodeHandle n;
  ros::Publisher p = n.advertise<arm_lib::angles_message>("angles_chatter", 1001);

  arm_lib::angles_message msg;

  std::string inputAngle1;
  std::string inputAngle2;
  std::string inputAngle3;
  std::string inputAngle4;

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    std::cout << "Input chasis_arm1_joint angle: ";
    std::getline(std::cin, inputAngle1);

    std::cout << "Input arm1_arm2_joint angle: ";
    std::getline(std::cin, inputAngle2);

    std::cout << "Input arm2_arm3_joint angle: ";
    std::getline(std::cin, inputAngle3);

    std::cout << "Input arm3_arm4_joint angle: ";
    std::getline(std::cin, inputAngle4);

    msg.a1 = std::stof(inputAngle1);
    msg.a2 = std::stof(inputAngle2);
    msg.a3 = std::stof(inputAngle3);
    msg.a4 = std::stof(inputAngle4);
    ROS_INFO("%f %f %f %f", msg.a1, msg.a2, msg.a3, msg.a4);
    p.publish(msg);

    ros::spinOnce();
  }

  return 0;
}
