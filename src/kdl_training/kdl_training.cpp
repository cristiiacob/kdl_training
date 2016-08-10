#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <sensor_msgs/JointState.h>
#include <kdl/frames_io.hpp>
#include <tf/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <kdl_conversions/kdl_msg.h>
#include <ros/console.h>

// TODO: separate this in hpp-file and cpp-file
// TODO: afterwards, write unit-tests for this
KDL::JntArray toKDL(const sensor_msgs::JointState::ConstPtr& msg, const std::vector<size_t>& indeces)
{
	KDL::JntArray result = KDL::JntArray(indeces.size());
  	
	for(int i = 0; i < indeces.size(); i++)
        {
        	result(i) = msg->position[indeces[i]];
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
	
     	using KDL::operator<<;
//	std::cout << result << std::endl;
        ROS_DEBUG_STREAM("FK: " << std::endl << result << std::endl);
        return result;
}


class Transform
{

    public:
        Transform(const ros::NodeHandle& nh): nh_(nh), base_frame_("base_link"), target_frame_("r_gripper_tool_frame"), once_(true)
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

		ROS_INFO("Now: %f", now.toSec());
		ROS_INFO("TF: %f, ", tf_pose.stamp_.toSec());
		geometry_msgs::TransformStamped tf_msg;
		tf::transformStampedTFToMsg(tf_pose, tf_msg);
        	KDL::Frame kdl_pose;
		tf::transformMsgToKDL(tf_msg.transform, kdl_pose);
         	using KDL::operator<<;
//	        std::cout << kdl_pose << std::endl;
		ROS_DEBUG_STREAM("TF: " << std::endl << kdl_pose << std::endl);

		return kdl_pose;
        }

	void createIndexMap(const sensor_msgs::JointState::ConstPtr& msg)
	{
		for(int i=0; i<msg->name.size(); i++)
		{
			joint_index_map_[msg->name[i]] = i;
		}	

		for(size_t i=0; i<joint_names_.size(); ++i)
		{
  			std::map<std::string,size_t>::const_iterator it = joint_index_map_.find(joint_names_[i]);
 		        if (it != joint_index_map_.end())
   				joint_indeces_.push_back(it->second);
  			else
    				ROS_ERROR("Could not find joint '%s'", joint_names_[i].c_str());
		}		
	}

	void chatterCallback(const sensor_msgs::JointState::ConstPtr& msg)
	{

	if(once_)
	{
		createIndexMap(msg);
		once_ = false;
	}

		KDL::Frame kdl_frame = calculateFK(chain_, toKDL(msg, joint_indeces_));
		KDL::Frame tf_frame = getTF(base_frame_, target_frame_);

		if (KDL::Equal(kdl_frame, tf_frame, 0.1))
		ROS_INFO("The conversion is correct!");
		else
		ROS_INFO("The conversion did not succed!");
	}

	void createChain(const std::string& robot_desc)
	{
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
	}
	
	void getJointNames()
	{
		ROS_INFO("Nr of joints: %d",chain_.getNrOfJoints());
		ROS_INFO("Total nr of segments: %d",chain_.getNrOfSegments());
		
		for(size_t i = 0; i < chain_.getNrOfSegments(); i++)
		{	
			KDL::Segment segment = chain_.getSegment(i);

		        if(segment.getJoint().getTypeName().compare("None"))
			{
				ROS_INFO("%s",segment.getJoint().getName().c_str());
				joint_names_.push_back(segment.getJoint().getName());	
			}				
		}	
	}
	
	void start()
	{			
                std::string robot_desc;
		nh_.param("robot_description", robot_desc, std::string());
		createChain(robot_desc);
		getJointNames();	
	 	sub_ = nh_.subscribe("joint_states", 1, &Transform::chatterCallback, this);
    	}	
	
    private:
	ros::NodeHandle nh_;
        tf::TransformListener listener_;
	ros::Subscriber sub_;
	std::string base_frame_;
	std::string target_frame_;
        KDL::Chain chain_; // TODO: remove this member, and replace it by the solver
	std::vector<size_t> joint_indeces_;
	std::vector<std::string> joint_names_;
	std::map<std::string, size_t> joint_index_map_;
	bool once_;
		
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

