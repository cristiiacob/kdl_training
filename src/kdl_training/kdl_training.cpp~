#include <kdl_training/kdl_transform.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv,"kdl_parser");
    	ros::NodeHandle nh;
	kdl_training::Transform my_transform(nh);

	try
        {
     	   	my_transform.start();
        }
        catch (const std::exception& ex)
        {
       		ROS_ERROR("%s",ex.what());
		return 0;
        }

	ros::spin();

    return 0;
}

