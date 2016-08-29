#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>  TrajClient;

class RobotArm
{
     private:
	// Action client for the joint trajectory action
	// used to trigger the arm movement action
	TrajClient* traj_client_arm_;
	TrajClient* traj_client_torso_;

     public:
	RobotArm()
	{
		traj_client_arm_ = new TrajClient("/r_arm_controller/follow_joint_trajectory", true);
		traj_client_torso_ = new TrajClient("/torso_controller/follow_joint_trajectory", true);

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

	~RobotArm()
	{
		delete traj_client_arm_;
		delete traj_client_torso_;
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
		goal.trajectory.points[0].positions[0] = 0.05;
		goal.trajectory.points[0].velocities.resize(1);
		goal.trajectory.points[0].velocities[0] = 0.0;
		goal.trajectory.points[0].time_from_start = ros::Duration(2.0);

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
		goal.trajectory.points.resize(2);
		int ind = 0;
		goal.trajectory.points[ind].positions.resize(7);
		goal.trajectory.points[ind].positions[0] = 0.0;
		goal.trajectory.points[ind].positions[1] = 0.0;
		goal.trajectory.points[ind].positions[2] = 0.0;
 		goal.trajectory.points[ind].positions[3] = 0.0;
		goal.trajectory.points[ind].positions[4] = 0.0;
		goal.trajectory.points[ind].positions[5] = 0.0;
		goal.trajectory.points[ind].positions[6] = 0.0;
		goal.trajectory.points[ind].velocities.resize(7);
		for (size_t j = 0; j < 7; ++j)
		{
			goal.trajectory.points[ind].velocities[j] = 0.0;
		}
		goal.trajectory.points[ind].time_from_start = ros::Duration(3.0);

		ind += 1;
		goal.trajectory.points[ind].positions.resize(7);
		goal.trajectory.points[ind].positions[0] = -0.3;
		goal.trajectory.points[ind].positions[1] = 0.2;
		goal.trajectory.points[ind].positions[2] = -0.1;
 		goal.trajectory.points[ind].positions[3] = -1.2;
		goal.trajectory.points[ind].positions[4] = 1.5;
		goal.trajectory.points[ind].positions[5] = -0.3;
		goal.trajectory.points[ind].positions[6] = 0.5;
		goal.trajectory.points[ind].velocities.resize(7);
		for (size_t j = 0; j < 7; ++j)
		{
			goal.trajectory.points[ind].velocities[j] = 0.0;
		}
		goal.trajectory.points[ind].time_from_start = ros::Duration(6.0);

		return goal;
	}

	actionlib::SimpleClientGoalState getArmState()
	{
		return traj_client_arm_->getState();
	}

		actionlib::SimpleClientGoalState getTorsoState()
	{
		return traj_client_torso_->getState();
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "r_arm_torso_controller_test");

	RobotArm arm;

	arm.startTrajectory(arm.armExtensionTrajectory(), arm.torsoExtensionTrajectory());
	// Wait for trajectory completion
	while (!arm.getArmState().isDone() && !arm.getTorsoState().isDone() && ros::ok())
	{
		usleep(50000);
	}
}
