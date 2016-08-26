#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>  TrajClient;

class RobotArm
{	
     private:
	// Action client for the joint trajectory action
	// used to trigger the arm movement action
	TrajClient* traj_client_elbow_flex_;
	TrajClient* traj_client_torso_;

      public:
	// Initialize the action client and wait for the action server to come up
	RobotArm()
	{
		// tell the action client that we want to spin a thread by default
		traj_client_elbow_flex_ = new TrajClient("r_elbow_flex_controller/follow_joint_trajectory", true);
		traj_client_torso_ = new TrajClient("/torso_lift_controller/follow_joint_trajectory", true);
		
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

	~RobotArm()
	{
		delete traj_client_elbow_flex_;
		delete traj_client_torso_;
	}

	void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal_elbow, control_msgs::FollowJointTrajectoryGoal goal_torso)
	{
		// When to start the trajectory: 'duration' from now
		goal_elbow.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
		goal_torso.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
		traj_client_elbow_flex_->sendGoal(goal_elbow);
		traj_client_torso_->sendGoal(goal_torso);
	}

	control_msgs::FollowJointTrajectoryGoal armExtensionTrajectory()
	{
		control_msgs::FollowJointTrajectoryGoal goal;
		goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
		goal.trajectory.points.resize(1);
		goal.trajectory.points[0].positions.resize(1);
		goal.trajectory.points[0].positions[0] = -0.1;
		goal.trajectory.points[0].velocities.resize(1);
		goal.trajectory.points[0].velocities[0] = 0.0;
		goal.trajectory.points[0].time_from_start = ros::Duration(2.0);

		return goal;
	}

	control_msgs::FollowJointTrajectoryGoal torsoExtensionTrajectory()
	{
		control_msgs::FollowJointTrajectoryGoal goal;
		goal.trajectory.joint_names.push_back("torso_lift_joint");
		goal.trajectory.points.resize(1);
		goal.trajectory.points[0].positions.resize(1);
		goal.trajectory.points[0].positions[0] = 0.05;
		goal.trajectory.points[0].velocities.resize(1);
		goal.trajectory.points[0].velocities[0] = 0.0;
		goal.trajectory.points[0].time_from_start = ros::Duration(2.0);

		return goal;
	}

	actionlib::SimpleClientGoalState getArmState()
	{
		return traj_client_elbow_flex_->getState();
	}

	actionlib::SimpleClientGoalState getTorsoState()
	{
		return traj_client_torso_->getState();
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "r_arm_test");

	RobotArm arm;

	arm.startTrajectory(arm.armExtensionTrajectory(), arm.torsoExtensionTrajectory());
	// Wait for trajectory completion
	while (!arm.getArmState().isDone() && !arm.getTorsoState().isDone() && ros::ok())
	{
		usleep(50000);
	}
}
