#include <arm_manager/arm_manager.h>
#include "sensor_msgs/JointState.h"


class StateMachineMiiwaHelper
{
	public:
	
		StateMachineMiiwaHelper(ArmManager* arm_manager, ArmManager* arm_manager_2 );
		~StateMachineMiiwaHelper();


		//Grasping method
		int GraspObject(std::vector<double> pose);

		void LeaveObject(int frame);

		void getDeliveryPlace(int frame, std::vector<double>& xyz, std::vector<double>& rpy);

		void moveSafePose();

		bool releaseProduct();

	private:
		
		//Arm and gripper manager
		ArmManager* arm_manager_;
		ArmManager* arm_manager_2_;

		ros::NodeHandle nh_;

		//FollowJointTrajectoryController topic subscriber and callback
		ros::Subscriber joint_state_sub_;
		void jointStatesCb(const sensor_msgs::JointState::ConstPtr& msg);
		sensor_msgs::JointState joint_state_;
		


		bool sync_;
		

};