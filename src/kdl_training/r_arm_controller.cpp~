#include <kdl_training/r_arm_controller.hpp>

int main(int argc, char** argv)
{
	// Init the ROS node
	ros::init(argc, argv, "r_arm_controller");
	ros::NodeHandle nh;

	kdl_training::RobotRArm my_arm(nh);
	// Start the trajectory
	
	
	// Wait for trajectory completion

	try
        {
     	   	my_arm.start();
        }
        catch (const std::exception& ex)
        {
       		ROS_ERROR("%s",ex.what());
		return 0;
        }

	ros::spin();
/*	while (!arm.getState().isDone() && ros::ok())
	{
		usleep(50000);
	}
*/
}
