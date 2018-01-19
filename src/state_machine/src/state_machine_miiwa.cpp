#include <ros/ros.h>
#include <eigen_functionalities.h>
#include "arm_manager/arm_manager.h"
#include "std_msgs/String.h"
#include "logimat_description/cam_info.h"
#include <logimat_description/state_machine_miiwa_helper.h>


enum StateMachine
{
	IDLE,
	PICKING,
	GO_TO_LEAVE_PLACE,
	LEAVE,
	BACK_HOME,
	
};

StateMachine state_;
ArmManager* arm_manager_;
ArmManager* arm_manager_2_;

StateMachineMiiwaHelper* state_machine_miiwa_helper_;


std::mutex state_mutex_;
bool initialization_done_ = false;

logimat_description::cam_info delivery_object_;

void chatterCallback(const logimat_description::cam_info msg)
{
	if(state_ == IDLE)
	{
		delivery_object_ = msg;
		ROS_INFO("NEW VALUE : %f", delivery_object_.or_y);
		state_ = PICKING;
	}

}

bool executeCycle()
{
	StateMachine state = state_;
	switch(state)
	{
		case IDLE:
		{
		ROS_INFO_THROTTLE(5,"In IDLE state");
		}
		break;
		
		case PICKING:
		{
			ROS_INFO_THROTTLE(5,"In PICKING state");
			std::vector<double> xyz(3,0), rpy(3,0);
			
			// HERE WE SHOULD ADD THE TRANSFORMATION: FROM CAM TO BASE

			xyz[0] = delivery_object_.x; xyz[1] = delivery_object_.y; xyz[2] = delivery_object_.z;
			rpy[0] = delivery_object_.or_x; rpy[1] = delivery_object_.or_y; rpy[2] = delivery_object_.or_z;

			std::vector<double> pose = {xyz[0],xyz[1],xyz[2],rpy[0],rpy[1],rpy[2]};

			int ret = state_machine_miiwa_helper_->GraspObject(pose);
			if(ret == -1)
			{
				state_machine_miiwa_helper_->moveSafePose();
				state_ = IDLE;
				ROS_INFO("Movement unreachable");
			}
			else
			{
				ROS_INFO("Movement finished");
				state_ = GO_TO_LEAVE_PLACE;
			}


			



		}
		break;
		case GO_TO_LEAVE_PLACE:
		{
			ROS_INFO_THROTTLE(5,"In GO_TO_LEAVE_PLACE state");

			int frame_id = std::stoi(delivery_object_.frame_id);

			std::vector<double> xyz(3,0), rpy(3,0);


			state_machine_miiwa_helper_->getDeliveryPlace(frame_id, xyz, rpy);


			arm_manager_2_->movePose(xyz, rpy);

			while(!arm_manager_2_->movementFinished())
			{
				//~ ROS_INFO("Arm moving");
				sleep(0.05);
			}
			
				ROS_INFO("Movement finished");
				state_ = LEAVE;
		}
		break;
		case LEAVE:
		{
			ROS_INFO_THROTTLE(5,"In LEAVING state");

			int frame_id = std::stoi(delivery_object_.frame_id);
			state_machine_miiwa_helper_->LeaveObject(frame_id);

			ROS_INFO("Movement finished");
			state_ = BACK_HOME;
		}
		break;
		case BACK_HOME:
		{
			ROS_INFO_THROTTLE(5,"In BACK_HOME state");

			std::vector<double> xyz(3,0), rpy(3,0);
			xyz[0] = 3.5;
			
			arm_manager_2_->movePose(xyz, rpy);
			while(!arm_manager_2_->movementFinished())
			{
				//~ ROS_INFO("Arm moving");
				sleep(0.05);
			}
			
				ROS_INFO("Movement finished");
				state_ = IDLE;
		}
		break;
	}

	return false;
}

void run_task()
{
	ros::NodeHandle nh;
	ros::Rate loop_rate(100);
	
	bool done = false;
	while(nh.ok())
	{
		done = executeCycle();
		if (done) 
			return;
			
		loop_rate.sleep();
	}
}


int main(int argc, char **argv)
{

	ros::init (argc, argv, "logimat_demo");
	ros::NodeHandle nh;
	
	ROS_INFO("Starting program...");
		
	ros::AsyncSpinner spinner(0);
	spinner.start();

	// Subscribe to the camera topic in order to receive information about the order.

	ros::Subscriber sub = nh.subscribe("cam_info", 1000, chatterCallback);

	
	// Init arm_manager & manipulation_helper
	ROS_INFO("PRE INITIALIZATION ... (JULEN)");

	arm_manager_ = new ArmManager("miiwa_arm", "lbr_iiwa_joint_trajectory_position_controller","arm");
	arm_manager_2_ = new ArmManager("miiwa_base", "omnirob_trajectory_position_controller","platform");
	
	ROS_INFO("well created -> INITIALIZATION ... (JULEN)");
	
	arm_manager_->initManager();
	arm_manager_2_->initManager();

	ROS_INFO("Arm manager initialized");
	
	state_machine_miiwa_helper_ = new StateMachineMiiwaHelper(arm_manager_, arm_manager_2_);

  	std::unique_lock<std::mutex> lock(state_mutex_);
  	initialization_done_ = false;
	state_ = IDLE;
	lock.unlock();
	
	run_task();
	 
	ros::waitForShutdown();

	ROS_INFO("Program finished");
   
  return 0;	


}
