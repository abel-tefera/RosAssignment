#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
// #include "arm_gazebo/jointangles.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "arm_lib/angles_message.h"
#include "arm_lib/xyz_coordinates.h"
#include "arm_lib/Fk.h"
#include "arm_lib/Ik.h"
#include <sstream>

namespace gazebo
{
	class ModelPush : public ModelPlugin
	{
	public:
		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
		{
			// Store the pointer to the model
			this->model = _parent;

			// // intiantiate the joint controller
			this->jointController = this->model->GetJointController();
			this->jointList = model->GetJoints();

			// // set your PID values
			this->pid = common::PID(30.1, 10.01, 10.03);

			for (int i = 0; i < jointList.size() - 2; i++)
			{
				jointController->SetPositionTarget(jointList[i]->GetScopedName(), 0);
				jointController->SetPositionPID(jointList[i]->GetScopedName(), this->pid);
			}

			this->pid2 = common::PID(100.0, 200.0, 100.0);

			jointController->SetPositionPID(model->GetJoint("arm6_arm7_joint")->GetScopedName(), this->pid2);
			jointController->SetPositionPID(model->GetJoint("arm6_arm8_joint")->GetScopedName(), this->pid2);

			jointController->SetPositionTarget(model->GetJoint("arm1_arm2_joint")->GetScopedName(), -0.4);
			jointController->SetPositionTarget(model->GetJoint("arm6_arm7_joint")->GetScopedName(), -0.7);
			jointController->SetPositionTarget(model->GetJoint("arm6_arm8_joint")->GetScopedName(), 0.7);

			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				std::bind(&ModelPush::OnUpdate, this));

			auto joint_name = "arm1_arm2_joint";

			std::string name = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();

			this->jointController->SetPositionPID(name, pid);

			int argc = 0;
			char **argv = NULL;
			// ros::init(argc, argv, "angle_listener");
			ros::init(argc, argv, "ros_control");

			//this->pub =n.advertise<arm_gazebo::jointangles>("pubangle_topic", 1000);
			// sub = n.subscribe("angle_topic", 1000, &ModelPush::setangles, this);
			positionsSub = n.subscribe("/xyz_coordinates", 1000, &ModelPush::coordinatesFun, this);
			fkClient = n.serviceClient<arm_lib::Fk>("fk");
			ikClient = n.serviceClient<arm_lib::Ik>("ik");
			ros::spinOnce();


		}

		void coordinatesFun(const arm_lib::xyz_coordinates &msg)
		{

			ROS_INFO("Received %f %f %f", msg.x, msg.y, msg.z);

			arm_lib::Ik srv;
			srv.request.arm_dim = {0.15, 2, 1, 0.5, 0.02, 0.02};
			srv.request.arr_jointPos = {0, 0, 0, 0, 0, 0};
			srv.request.xyz_coordinates = {msg.x, msg.y, msg.z};

			if (ikClient.call(srv))
			{

				ROS_INFO("Ik: [%f, %f, %f, %f, %f, %f]", srv.response.final_value[0], srv.response.final_value[1], srv.response.final_value[2], srv.response.final_value[3], srv.response.final_value[4], srv.response.final_value[5]);

				for (int i = 0; i < jointList.size() - 2; i++)
				{
					jointController->SetPositionTarget(jointList[i]->GetScopedName(), srv.response.final_value[i]);
				}
			}
			else
			{
				ROS_ERROR("Failed to call service ik");
			}
		}

		// void setangles(const arm_gazebo::jointangles::ConstPtr &msg)
		// {
		// 	std::cout << "Angles Recieved" << std::endl;
		// 	ROS_INFO("Angles: [%.2f]", msg->joint1);

		// 	std::string jointname1 = this->model->GetJoint("chasis_arm1_joint")->GetScopedName();
		// 	std::string jointname2 = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();
		// 	std::string jointname3 = this->model->GetJoint("arm2_arm3_joint")->GetScopedName();
		// 	std::string jointname4 = this->model->GetJoint("arm3_arm4_joint")->GetScopedName();

		// 	this->jointController->SetPositionTarget(jointname1, msg->joint1);
		// 	this->jointController->SetPositionTarget(jointname2, msg->joint2);
		// 	this->jointController->SetPositionTarget(jointname3, msg->joint3);
		// 	this->jointController->SetPositionTarget(jointname4, msg->joint4);
		// }

		void GetFk(double a1, double a2, double a3, double a4, double a5, double a6)
		{
			arm_lib::Fk srv;
			srv.request.arm_dim = {0.05, 2, 1, 0.5, 0.02, 0.02};
			srv.request.arr_jointPos = {a1, a2, a3, a4, a5, a6};

			if (fkClient.call(srv))
			{
				ROS_INFO("Fk: [%f, %f, %f]", srv.response.position[0], srv.response.position[1], srv.response.position[2]);
			}
			else
			{
				ROS_ERROR("Failed to call service fk");
			}
		}

		// Called by the world update start event
	public:
		void OnUpdate()
		{
			float angleDegree = -90;
			float rad = M_PI * angleDegree / 180;

			std::string name = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();
			std::string name2 = this->model->GetJoint("arm2_arm3_joint")->GetScopedName();
			std::string name3 = this->model->GetJoint("arm3_arm4_joint")->GetScopedName();

			// this->jointController->SetPositionPID(name, pid);
			// this->jointController->SetPositionTarget(name, rad);
			// this->jointController->Update();

			// Get joint position by index.
			// 0 returns rotation accross X axis
			// 1 returns rotation accross Y axis
			// 2 returns rotation accross Z axis
			// If the Joint has only Z axis for rotation, 0 returns that value and 1 and 2 return nan
			double a1 = physics::JointState(this->model->GetJoint("arm1_arm2_joint")).Position(0);
			double a2 = physics::JointState(this->model->GetJoint("arm2_arm3_joint")).Position(0);
			double a3 = physics::JointState(this->model->GetJoint("arm3_arm4_joint")).Position(0);
			double a4 = physics::JointState(this->model->GetJoint("chasis_arm1_joint")).Position(0);
			double a5 = physics::JointState(this->model->GetJoint("arm4_arm5_joint")).Position(0);
			double a6 = physics::JointState(this->model->GetJoint("arm5_arm6_joint")).Position(0);

			// double a2 = this->model->GetJoint("chasis_arm1_joint").Position(0);
			// double a3 = physics::JointState(this->model->GetJoint("chasis_arm1_joint")).Position(2);
			// std::cout << "Current chasis_arm1_joint values: " << a4 * 180.0 / M_PI << std::endl;
			// std::cout << "Current arm1_arm2_joint values: " << a1 * 180.0 / M_PI << std::endl;
			// std::cout << "Current arm2_arm3_joint values: " << a2 * 180.0 / M_PI << std::endl;
			// std::cout << "Current arm3_arm4_joint values: " << a3 * 180.0 / M_PI << std::endl;
			ros::spinOnce();
		}

		// a pointer that points to a model object
	private:
		physics::ModelPtr model;

		// 	// A joint controller object
		// 	// Takes PID value and apply angular velocity
		// 	//  or sets position of the angles

	private:
		ros::NodeHandle n;
		ros::Publisher pub;
		ros::Subscriber sub;
		ros::Subscriber sub2;
		ros::Subscriber positionsSub;
		ros::ServiceClient fkClient;
		ros::ServiceClient ikClient;

	private:
		physics::JointControllerPtr jointController;
		physics::Joint_V jointList;

	private:
		event::ConnectionPtr updateConnection;

		// // 	// PID object
	private:
		common::PID pid;
		common::PID pid2;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}