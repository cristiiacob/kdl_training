#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <sensor_msgs/JointState.h>
#include <kdl/frames_io.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <kdl_conversions/kdl_msg.h>

KDL::JntArray toKDL(const sensor_msgs::JointState& msg, const std::vector<size_t>& indeces)
{
	KDL::JntArray result = KDL::JntArray(indeces.size());
  	
	for(int i = 0; i < indeces.size(); i++)
        {
        	result(i) = msg.position[indeces[i]];
  	}
	
  	return result;
}

KDL::Frame calculateFK(const KDL::Chain& chain, const KDL::JntArray& joints)
{	
	KDL::ChainFkSolverPos_recursive my_solver(chain);
       	if(joints.rows() != chain.getNrOfJoints())
	{	
		throw std::logic_error( "Could not construct the tree" );
	}
	
        KDL::Frame result;
	my_solver.JntToCart(joints, result);
	ROS_INFO("FK:");
     	using KDL::operator<<;
	std::cout << result << std::endl;
        return result;
}


class Transform
{

    public:
        Transform(const ros::NodeHandle& nh): nh_(nh), base_frame_("base_link"), target_frame_("r_gripper_tool_frame")
        {}

        ~Transform() {}
	

	KDL::Frame getTF(const std::string& frame_id, const std::string& child_frame_id)
        {
		tf::StampedTransform tf_pose;
		ros::Time now = ros::Time::now();
		if (listener_.waitForTransform(frame_id, child_frame_id, now, ros::Duration(0.1)))
                	listener_.lookupTransform(frame_id, child_frame_id, now, tf_pose);
		else
                	ROS_ERROR("Could not look up transform.");

ROS_INFO("Now: %f", ros::Time::now().toSec());
ROS_INFO("TF: %f, ", tf_pose.stamp_.toSec());
		geometry_msgs::TransformStamped tf_msg;
		tf::transformStampedTFToMsg(tf_pose, tf_msg);
        	KDL::Frame kdl_pose;
		tf::transformMsgToKDL(tf_msg.transform, kdl_pose);
		ROS_INFO("TF:");
         	using KDL::operator<<;
	        std::cout << kdl_pose << std::endl;

		return kdl_pose;
        }

	void chatterCallback(sensor_msgs::JointState msg)
	{
		std::vector<size_t> indeces;
		indeces.push_back(12);
		indeces.push_back(18);
		indeces.push_back(19);
		indeces.push_back(17);
		indeces.push_back(21);
		indeces.push_back(20);
		indeces.push_back(22);
		indeces.push_back(23);

		KDL::Frame kdl_frame = calculateFK(chain_, toKDL(msg, indeces));
		KDL::Frame tf_frame = getTF(base_frame_, target_frame_);

		if (KDL::Equal(kdl_frame, tf_frame, 0.1))
		ROS_INFO("The conversion is correct!");
		else
		ROS_INFO("The conversion did not succed!");

/*		int size = msg.name.size();
		ROS_INFO("size: [%d]", size);	

		for (int i = 0; i < size; i++)
		{
			if (msg.name[i].compare("r_shoulder_pan_joint") == 0);
			{	
				ROS_INFO("i: [%d]", i);	
				ROS_INFO("I heard: [%s]", msg.name[i].c_str());	
				//break;
			}
		} 
*/		
	}

	
	void start()
	{			
                std::string robot_desc;
		nh_.param("robot_description", robot_desc, std::string());
		KDL::Tree my_tree;
		if (!kdl_parser::treeFromString(robot_desc, my_tree))
		{
			ROS_ERROR("Failed to construct kdl tree");
			throw std::logic_error( "Could not construct the tree" );
                        return;
		}
	
		if (!my_tree.getChain(base_frame_, target_frame_, chain_))
		{
			ROS_ERROR("Failed to get chain form KDL tree.");
                        throw std::logic_error( "Could not construct the chain" );
                        return;
		}
		ROS_INFO("%d",chain_.getNrOfJoints());

	 	sub_ = nh_.subscribe("joint_states", 1, &Transform::chatterCallback, this);
    	}	
	
    private:
	ros::NodeHandle nh_;
        tf::TransformListener listener_;
	ros::Subscriber sub_;
	std::string base_frame_;
	std::string target_frame_;
        KDL::Chain chain_;
		
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

