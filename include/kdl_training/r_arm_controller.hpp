#ifndef KDL_TRAINING_R_ARM_CONTROLLER_HPP
#define KDL_TRAINING_R_ARM_CONTROLLER_HPP

#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float32MultiArray.h>

namespace kdl_training
{
	typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;
	typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;

	class RobotRArm
	{
	   private:
		TrajClient *traj_client_;
		TorsoClient *torso_client_;
		ros::NodeHandle nh_;
		ros::Subscriber sub_goal_;
		std::vector<float> goal_points_;
		
	   public:
		// Initialize the action client and wait for the action server to come up
		RobotRArm(const ros::NodeHandle& nh): nh_(nh)
		{
			// tell the action client that we want to spin a thread by default
			traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);
			torso_client_ = new TorsoClient("torso_controller/position_joint_action", true);
		
			// wait for action server to come up
			while (!traj_client_->waitForServer(ros::Duration(1.0)) && !torso_client_->waitForServer(ros::Duration(1.0)))
			{
				ROS_INFO("Waiting for the action serer to come up");
			}
		}
	
		~RobotRArm()
		{
			delete traj_client_;
		}
	
		void callback(const std_msgs::Float32MultiArray& msg)
		{
			for (size_t  i = 0; i < 8; ++i)
				goal_points_.push_back(msg.data[i]); 	
		}

		void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal arm_goal, pr2_controllers_msgs::SingleJointPositionGoal torso_goal)
		{	
			while(goal_points_.empty())
				ros::Duration(1).sleep();
			
			// When to start the trajectory: 'duration' from now
			//arm_goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
			torso_client_->sendGoal(torso_goal);
			//torso_client_->waitForResult(ros::Duration(7.0));
			traj_client_->sendGoal(arm_goal);
		}
		
		pr2_controllers_msgs::SingleJointPositionGoal torsoLift()
		{
			pr2_controllers_msgs::SingleJointPositionGoal goal;
			goal.position = goal_points_[0];
			goal.position = 0.1;
			goal.min_duration = ros::Duration(1.0);	
			goal.max_velocity = 1.0;
			ROS_INFO("Sending torso goal");

			return goal;			
		}
	
		pr2_controllers_msgs::JointTrajectoryGoal armExtensionTrajectory()
		{
			pr2_controllers_msgs::JointTrajectoryGoal goal;
			goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
			goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
			goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
			goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
			goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
			goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
			goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
			goal.trajectory.points.resize(1);

			goal.trajectory.points[0].positions.resize(7);
			for (size_t i = 0; i < 7; ++i)
				goal.trajectory.points[0].positions[i] = goal_points_[i+1];
				//goal.trajectory.points[0].positions[i] = 0.3;

			goal.trajectory.points[0].velocities.resize(7);
			for (size_t j = 0; j < 7; ++j)
			{
				goal.trajectory.points[0].velocities[j] = 0.0;
			}

			goal.trajectory.points[0].time_from_start = ros::Duration(3.0);
			ROS_INFO("Sending arm goal");

			return goal;
		}	

		// Returns the current state of the action
		actionlib::SimpleClientGoalState getState()
		{
			return traj_client_->getState();
		}
	};
}

#endif // KDL_TRAINING_R_ARM_CONTROLLER_HPP
