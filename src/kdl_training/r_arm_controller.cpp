#include <kdl_training/r_arm_controller.hpp>

int main(int argc, char** argv)
{
	// Init the ROS node
	ros::init(argc, argv, "r_arm_controller");
	ros::NodeHandle nh;

	kdl_training::RobotRArm arm(nh);
	// Start the trajectory
	arm.startTrajectory(arm.armExtensionTrajectory(), arm.torsoLift());
	// Wait for trajectory completion

	ros::spin();
/*	while (!arm.getState().isDone() && ros::ok())
	{
		usleep(50000);
	}
*/
}
