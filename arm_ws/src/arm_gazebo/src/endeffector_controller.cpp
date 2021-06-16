#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "arm_lib/cmd_message.h"

#include <sstream>

namespace gazebo
{
    class EndeffectorController : public ModelPlugin
    {

    public:
        void commandCallback(const arm_lib::cmd_message &msg)
        {

            ROS_INFO("Received %s", msg.command_name.c_str());

            std::string command = msg.command_name;

            std::string arm6_arm8 = this->model->GetJoint("arm6_arm8_joint")->GetScopedName();
            std::string arm6_arm7 = this->model->GetJoint("arm6_arm7_joint")->GetScopedName();

            float toRad = M_PI / 180.0;

            if (command == "catch")
            {

                this->jointController->SetPositionTarget(arm6_arm8, -60 * toRad);
                this->jointController->SetPositionTarget(arm6_arm7, 60 * toRad);
            }

            else if (command == "release")
            {

                this->jointController->SetPositionTarget(arm6_arm8, 0 * toRad);
                this->jointController->SetPositionTarget(arm6_arm7, 0 * toRad);
            }
        }
        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {
            this->model = _parent;
            this->jointController = this->model->GetJointController();

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&EndeffectorController::OnUpdate, this));

            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "ros_control");
            sub = n.subscribe("/command_chatter", 1000, &EndeffectorController::commandCallback, this);
            ros::spinOnce();
        }

        void OnUpdate()
        {
            ros::spinOnce();
        }

    private:
        physics::ModelPtr model;

        physics::JointControllerPtr jointController;

        event::ConnectionPtr updateConnection;

        ros::NodeHandle n;

        ros::Subscriber sub;
    };

    GZ_REGISTER_MODEL_PLUGIN(EndeffectorController)
}