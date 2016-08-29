#include <kdl_training/kdl_keyboard.hpp>
#include <stdlib.h> 

int main(int argc, char **argv)
{
   	ros::init(argc, argv, "tf_goal");
	ros::NodeHandle nh("~");
		
	if (argc != 8)
		throw std::runtime_error("Give 7 floats! Pos and Quat");
    	geometry_msgs::Pose tf_pose;
	tf_pose.position.x = strtod(argv[1], NULL);
	tf_pose.position.y = strtod(argv[2], NULL);
	tf_pose.position.z = strtod(argv[3], NULL);
	tf_pose.orientation.x = strtod(argv[4], NULL);
	tf_pose.orientation.y = strtod(argv[5], NULL);
	tf_pose.orientation.z = strtod(argv[6], NULL);
	tf_pose.orientation.w = strtod(argv[7], NULL);

    	kdl_training::TFGoal my_tf_goal(nh, tf_pose);

   	 try
   	 {
   //	    my_tf_goal.start(ros::Duration(10.0));
	    my_tf_goal.start();
   	 }
   	 catch(const std::exception& e)
   	 {
   	   ROS_ERROR("%s", e.what());
   	   return 0;
   	 }
	
        ros::spinOnce();
	//ros::spin();
	
   	 return 0;
}
