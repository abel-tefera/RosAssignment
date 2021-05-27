#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <iostream>
#include "arm_gazebo/jointangles.h"
#include "ros/ros.h"

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

			// // set your PID values
			this->pid = common::PID(30.1, 10.01, 10.03);

			auto joint_name = "arm1_arm2_joint";

			std::string name = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();

			this->jointController->SetPositionPID(name, pid);

			int argc = 0;
			char **argv = nullptr;
			ros::init(argc, argv, "angle_listener");

			ros::NodeHandle n;
			//this->pub =n.advertise<arm_gazebo::jointangles>("pubangle_topic", 1000);
			this->sub = n.subscribe("angle_topic", 1000, &ModelPush::setangles, this);

			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				std::bind(&ModelPush::OnUpdate, this));
		}

		void setangles(const arm_gazebo::jointangles::ConstPtr &msg)
		{
			std::cout << "Angles Recieved" << std::endl;
			ROS_INFO("Angles: [%.2f]", msg->joint1);

			std::string jointname1 = this->model->GetJoint("chasis_arm1_joint")->GetScopedName();
			std::string jointname2 = this->model->GetJoint("arm1_arm2_joint")->GetScopedName();
			std::string jointname3 = this->model->GetJoint("arm2_arm3_joint")->GetScopedName();
			std::string jointname4 = this->model->GetJoint("arm3_arm4_joint")->GetScopedName();

			this->jointController->SetPositionTarget(jointname1, msg->joint1);
			this->jointController->SetPositionTarget(jointname2, msg->joint2);
			this->jointController->SetPositionTarget(jointname3, msg->joint3);
			this->jointController->SetPositionTarget(jointname4, msg->joint4);
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
			// double a2 = this->model->GetJoint("chasis_arm1_joint").Position(0);
			// double a3 = physics::JointState(this->model->GetJoint("chasis_arm1_joint")).Position(2);
			std::cout << "Current chasis_arm1_joint values: " << a4 * 180.0 / M_PI << std::endl;
			std::cout << "Current arm1_arm2_joint values: " << a1 * 180.0 / M_PI << std::endl;
			std::cout << "Current arm2_arm3_joint values: " << a2 * 180.0 / M_PI << std::endl;
			std::cout << "Current arm3_arm4_joint values: " << a3 * 180.0 / M_PI << std::endl;
		}

		// a pointer that points to a model object
	private:
		physics::ModelPtr model;

		// 	// A joint controller object
		// 	// Takes PID value and apply angular velocity
		// 	//  or sets position of the angles

	private:
		ros::Publisher pub;
		ros::Subscriber sub;

	private:
		physics::JointControllerPtr jointController;

	private:
		event::ConnectionPtr updateConnection;

		// // 	// PID object
	private:
		common::PID pid;
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}