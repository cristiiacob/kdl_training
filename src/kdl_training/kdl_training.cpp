#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <sensor_msgs/JointState.h>
#include <kdl/frames_io.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <kdl_conversions/kdl_msg.h>



class Transform
{

    public:
        Transform(const ros::NodeHandle& nh): nh_(nh)
        {}

        ~Transform() {}
	
	void convertChain()
	{	
		nh_.param("robot_description", robot_desc_string_, std::string());
		KDL::Tree my_tree;
		if (!kdl_parser::treeFromString(robot_desc_string_, my_tree))
		{
			ROS_ERROR("Failed to construct kdl tree");
			exit(-1);
		}
	
		KDL::Chain my_chain;
		if (!my_tree.getChain("base_link","l_gripper_tool_frame", my_chain))
		{
			ROS_ERROR("Failed to get chain form KDL tree.");
			exit(-1);
		}
	
		KDL::ChainFkSolverPos_recursive my_solver(my_chain);
		KDL::JntArray jnt_pos_;
       		jnt_pos_.resize(my_chain.getNrOfJoints());	
	
		my_solver.JntToCart(jnt_pos_, current_pose_);
     		using KDL::operator<<;
		std::cout << current_pose_ << std::endl;
	}

	bool listenAndRespond(std::string frame_id_arg, std::string child_frame_id_arg)
        {
      		std::string frame_id = frame_id_arg;
        	std::string child_frame_id = child_frame_id_arg;
		bool isEqual = KDL::Equal(calculated_pose_, current_pose_, 0.7);

        	ROS_INFO("Parent %s", frame_id.c_str());
        	ROS_INFO("Child %s", child_frame_id.c_str());
        	try
                { 	
			listener_.waitForTransform(frame_id, child_frame_id, ros::Time(0), ros::Duration(0.5));
                	listener_.lookupTransform(frame_id, child_frame_id, ros::Time(0), transformed_);
	        }
                catch (tf::TransformException& ex)
                {
                	ROS_ERROR("%s", ex.what());
                }
		
	        tf::transformStampedTFToMsg(transformed_, msg_);
	        tf::transformMsgToKDL(msg_.transform, calculated_pose_);
	
	        return isEqual;
        }

	void chatterCallback(sensor_msgs::JointState msg)
	{
		convertChain();
		bool equal = listenAndRespond("base_link", "l_gripper_tool_frame");

		if (equal)
		ROS_INFO("The conversion is correct!");
		else
		ROS_INFO("The conversion did not succed!");

	//	int size = msg.name.size();
	//	ROS_INFO("size: [%d]", size);	

/*		for (int i = 0; i < size; i++)
		{
			if (msg.name[i].compare("r_shoulder_pan_joint") == 0);
			{	
				ROS_INFO("i: [%d]", i);	
				ROS_INFO("I heard: [%s]", msg.name[i].c_str());	
				//break;
			}
		} 
*/		

	//	ROS_INFO("I heard: [%s], position: [%f]", msg.name[18].c_str(), msg.position[18]);
	//	ROS_INFO("I heard: [%s], position: [%f]", msg.name[19].c_str(), msg.position[19]);
	//	ROS_INFO("I heard: [%s], position: [%f]", msg.name[17].c_str(), msg.position[17]);
	//	ROS_INFO("I heard: [%s], position: [%f]", msg.name[21].c_str(), msg.position[21]);
	//	ROS_INFO("I heard: [%s], position: [%f]", msg.name[20].c_str(), msg.position[20]);
	//	ROS_INFO("I heard: [%s], position: [%f]", msg.name[22].c_str(), msg.position[22]);
	//	ROS_INFO("I heard: [%s], position: [%f]", msg.name[23].c_str(), msg.position[23]);
	}

	
	void start()
	{			
	 	sub_ = nh_.subscribe("joint_states", 1, &Transform::chatterCallback, this);
    	}	
	
    private:
	ros::NodeHandle nh_;
        tf::TransformListener listener_;
	tf::StampedTransform transformed_;
	ros::Subscriber sub_;
	KDL::Frame calculated_pose_;	
	geometry_msgs::TransformStamped msg_;
	KDL::Frame current_pose_;
	std::string robot_desc_string_;	
		
};

int main(int argc, char **argv)
{
	ros::init(argc, argv,"kdl_parser");
    	ros::NodeHandle nh;
	Transform my_transform(nh);

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

