#ifndef KDL_TRAINING_R_ARM_CONTROLLER_HPP
#define KDL_TRAINING_R_ARM_CONTROLLER_HPP

#include <ros/ros.h>
//#include <pr2_controllers_msgs/JointTrajectoryAction.h>
//#include <pr2_controllers_msgs/SingleJointPositionAction.h>
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
			traj_client_elbow_flex_ = new TrajClient("r_elbow_flex_controller/follow_joint_trajectory", true);
			traj_client_torso_ = new TrajClient("/torso_lift_controller/follow_joint_trajectory", true);
goal_points_.resize(8);
			goal_points_.resize(2);
		
			// wait for action server to come up
			while (!traj_client_elbow_flex_->waitForServer(ros::Duration(5.0)))
		{
			ROS_INFO("Waiting for the r_elbow_flex_controller/follow_joint_trajectory server");
		}

		while (!traj_client_torso_->waitForServer(ros::Duration(5.0)))
		{
			ROS_INFO("Waiting for the r_elbow_flex_controller/follow_joint_trajectory server");
		}
		ROS_INFO("Works for now");
		}
	
		~RobotRArm()
		{
			delete traj_client_elbow_flex_;
			delete traj_client_torso_;
		}
	
		void callback(const std_msgs::Float32MultiArray& msg)
		{	
			std::vector<float> test_vector;
			for (size_t  i = 0; i < 8; ++i)
			{
				test_vector.push_back(msg.data[i]); 	
				ROS_INFO("%f", msg.data[i]);
			}
			if(test_vector != goal_points_)
			{	
				for (size_t  i = 0; i < 8; ++i)
				{
					goal_points_[i] = msg.data[i]; 	
					ROS_INFO("%f", msg.data[i]);
				}
				startTrajectory(armExtensionTrajectory(), torsoExtensionTrajectory());
			}
		}

		void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal_elbow, control_msgs::FollowJointTrajectoryGoal goal_torso)
		{
			// When to start the trajectory: 'duration' from now
			goal_elbow.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
			goal_torso.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
			traj_client_elbow_flex_->sendGoal(goal_elbow);
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
			goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
			goal.trajectory.points.resize(1);
			goal.trajectory.points[0].positions.resize(1);
			goal.trajectory.points[0].positions[0] = goal_points_[1];
			goal.trajectory.points[0].velocities.resize(1);
			goal.trajectory.points[0].velocities[0] = 0.0;
			goal.trajectory.points[0].time_from_start = ros::Duration(2.0);
			ROS_INFO("Sending elbow goal");

			return goal;
		}
	

		// Returns the current state of the action
		actionlib::SimpleClientGoalState getArmState()
		{
			return traj_client_elbow_flex_->getState();
		}

		actionlib::SimpleClientGoalState getTorsoState()
		{
			return traj_client_torso_->getState();
		}

		void start()
		{
			sub_goal_ = nh_.subscribe("/goal_joint_pos", 1, &RobotRArm::callback, this);

		//	while(goal_points_.empty())
		//	{
		//		ros::Duration(1).sleep();
		//		ROS_INFO("Waiting for msg");
		//	}		
			
		}

	    private:
		TrajClient* traj_client_elbow_flex_;
		TrajClient* traj_client_torso_;
		ros::NodeHandle nh_;
		ros::Subscriber sub_goal_;
		std::vector<float> goal_points_;
		ros::Subscriber sub_;
	};
}

#endif // KDL_TRAINING_R_ARM_CONTROLLER_HPP
