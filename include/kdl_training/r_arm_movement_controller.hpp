#ifndef KDL_TRAINING_R_ARM_MOVEMENT_CONTROLLER_HPP
#define KDL_TRAINING_R_ARM_MOVEMENT_CONTROLLER_HPP

#include <ros/ros.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float32MultiArray.h>

namespace kdl_training
{
	typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>  TrajClient;

	class RobotRArm
	{	
	   public:
		// Initialize the action client and wait for the action server to come up
		RobotRArm(const ros::NodeHandle& nh): nh_(nh)
		{
			// tell the action client that we want to spin a thread by default
			traj_client_arm_ = new TrajClient("/r_arm_controller/follow_joint_trajectory", true);
			traj_client_torso_ = new TrajClient("/torso_controller/follow_joint_trajectory", true);
			goal_points_.resize(8);
		
			// wait for action server to come up
			while (!traj_client_arm_->waitForServer(ros::Duration(5.0)))
			{
				ROS_INFO("Waiting for the /r_arm_controller/follow_joint_trajectory server");
			}

			while (!traj_client_torso_->waitForServer(ros::Duration(5.0)))
			{
				ROS_INFO("Waiting for the /torso_controller/follow_joint_trajectory server");
			}

			ROS_INFO("Works for now");
		}

		~RobotRArm()
		{
			delete traj_client_arm_;
			delete traj_client_torso_;
		}

		void callback(const std_msgs::Float32MultiArray& msg)
		{	
			/*std::vector<float> test_vector;	
			for (size_t  i = 0; i < 8; ++i)
			{
				test_vector.push_back(msg.data[i]); 	
				ROS_INFO("%f", msg.data[i]);
			}
			if(test_vector != goal_points_)
			{	
				ROS_INFO("It was not the same!");
				for (size_t  i = 0; i < 8; ++i)
				{
					goal_points_[i] = msg.data[i]; 	
					ROS_INFO("%f", msg.data[i]);
				}
				startTrajectory(armExtensionTrajectory(), torsoExtensionTrajectory());
			}
*/

				for (size_t  i = 0; i < 8; ++i)
				{
					goal_points_[i] = msg.data[i]; 	
					ROS_INFO("%f", msg.data[i]);
				}
				startTrajectory(armExtensionTrajectory(), torsoExtensionTrajectory());
		}

		void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal_arm, control_msgs::FollowJointTrajectoryGoal goal_torso)
		{
			// When to start the trajectory: 'duration' from now
			goal_arm.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
			goal_torso.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
			traj_client_arm_->sendGoal(goal_arm);
			traj_client_torso_->sendGoal(goal_torso);
		}
		
		control_msgs::FollowJointTrajectoryGoal torsoExtensionTrajectory()
		{
			control_msgs::FollowJointTrajectoryGoal goal;
			goal.trajectory.joint_names.push_back("torso_lift_joint");
			goal.trajectory.points.resize(1);
			goal.trajectory.points[0].positions.resize(1);
			goal.trajectory.points[0].positions[0] = goal_points_[0];
			goal.trajectory.points[0].velocities.resize(1);
			goal.trajectory.points[0].velocities[0] = 0.0;
			goal.trajectory.points[0].time_from_start = ros::Duration(2.0);
			ROS_INFO("Sending torso goal");

			return goal;
		}

		control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory()
		{
			control_msgs::FollowJointTrajectoryGoal goal;
			goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
			goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
			goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
			goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
			goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
			goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
			goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
			goal.trajectory.points.resize(1);
			int ind = 0;
			goal.trajectory.points[ind].positions.resize(7);
			goal.trajectory.points[ind].positions[0] = goal_points_[1];
			goal.trajectory.points[ind].positions[1] = goal_points_[2];
			goal.trajectory.points[ind].positions[2] = goal_points_[3];
	 		goal.trajectory.points[ind].positions[3] = goal_points_[4];
			goal.trajectory.points[ind].positions[4] = goal_points_[5];
			goal.trajectory.points[ind].positions[5] = goal_points_[6];
			goal.trajectory.points[ind].positions[6] = goal_points_[7];
			goal.trajectory.points[ind].velocities.resize(7);
			for (size_t j = 0; j < 7; ++j)
			{
				goal.trajectory.points[ind].velocities[j] = 0.0;
			}
			goal.trajectory.points[ind].time_from_start = ros::Duration(3.0);
			ROS_INFO("Sending arm goal");

			return goal;
		}
	
		// Returns the current state of the action
		actionlib::SimpleClientGoalState getArmState()
		{
			return traj_client_arm_->getState();
		}

		actionlib::SimpleClientGoalState getTorsoState()
		{
			return traj_client_torso_->getState();
		}

		void start()
		{
			sub_goal_ = nh_.subscribe("/goal_joint_pos", 1, &RobotRArm::callback, this);

			while(goal_points_.empty())
			{
				ros::Duration(1).sleep();
				ROS_INFO("Waiting for msg");
			}				
		}

	    private:
		TrajClient* traj_client_arm_;
		TrajClient* traj_client_torso_;
		ros::NodeHandle nh_;
		ros::Subscriber sub_goal_;
		std::vector<float> goal_points_;
		ros::Subscriber sub_;
	};
}

#endif // KDL_TRAINING_R_ARM_MOVEMENT_CONTROLLER_HPP
