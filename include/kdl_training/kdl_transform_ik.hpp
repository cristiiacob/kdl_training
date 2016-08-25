#ifndef KDL_TRAINING_KDL_TRANSFORM_IK_HPP
#define KDL_TRAINING_KDL_TRANSFORM_IK_HPP

#include <ros/ros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_training/kdl_functions.hpp>
#include <kdl/frames_io.hpp>
#include <geometry_msgs/Pose.h>
#include "kdl_conversions/kdl_msg.h"
#include <std_msgs/Float32MultiArray.h>

namespace kdl_training
{
	class Transform_ik
	{
	    public:
		Transform_ik(const ros::NodeHandle& nh): nh_(nh), base_frame_("base_link"), target_frame_("r_gripper_tool_frame"), once_(true)
		{}

		~Transform_ik() {}
		
		void stateCallback(const sensor_msgs::JointState::ConstPtr& msg)
		{
                     	last_joint_state_ = *msg;
		
			if(once_)
			{
				joint_indeces_ = createJointIndices(createIndexMap(msg, joint_names_), joint_names_);
				once_ = false;
			}
		//	q_out_ = calculateIK(ik_solver_, goal_frame_, toKDL(*msg, joint_indeces_));	
		/*	using KDL::operator<<;
			ROS_DEBUG_STREAM("IK: " << std::endl << q_out_ << std::endl);
			for (size_t i = 0; i < q_out_.rows(); ++i)		
				ROS_INFO("%f",q_out_(i)); 
		*/		
		}

		void goalCallback(const geometry_msgs::Pose::ConstPtr& msg)
		{	
// TODO: print input msg
			tf::poseMsgToKDL(*msg, goal_frame_);		
			if(!last_joint_state_.position.empty())
                        {
				q_out_ = calculateIK(ik_solver_, goal_frame_, toKDL(last_joint_state_, joint_indeces_));
			
				std_msgs::Float32MultiArray pub_msg;
				for(int i = 0; i< q_out_.rows(); ++i)
				{
					ROS_INFO("%f", q_out_(i));
					pub_msg.data.push_back(q_out_(i));
				
				}
				pub_.publish(pub_msg);	
                        }
			else
				ROS_WARN("Could not process goal because there was not JointStates, yet.");
		}
	
		void createChain(const std::string& robot_desc)
		{
			KDL::Tree my_tree;
		
			if (!kdl_parser::treeFromString(robot_desc, my_tree))
		        	throw std::runtime_error( "Could not construct the tree" );


			if (!my_tree.getChain(base_frame_, target_frame_, chain_))
		                throw std::runtime_error( "Could not construct the chian" );
		}

		void start()
		{			
		        std::string robot_desc;
			nh_.param("robot_description", robot_desc, std::string());
			createChain(robot_desc);
			ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(KDL::ChainIkSolverPos_LMA(chain_));	
			joint_names_ = getJointNames(chain_);
			
			for(size_t i = 0; i <joint_names_.size(); ++i)
				ROS_INFO("%s", joint_names_[i].c_str());	
			
			sub_state_ = nh_.subscribe("joint_states", 1, &Transform_ik::stateCallback, this);
			
			try
			{
				ros::Duration(2).sleep();
			}			
			catch(std::exception& e)
			{
				ROS_ERROR("%s",e.what());	
			}
					
		 	sub_goal_ = nh_.subscribe("/tf_goal", 1, &Transform_ik::goalCallback, this);	
			pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/goal_joint_pos", 1);
        		//timer_ = nh_.createTimer(period, &Transform_ik::goalPublishCallback, this);
	    	}
	
	    private:
		ros::NodeHandle nh_;
		std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
		std::string base_frame_;
		std::string target_frame_;
		KDL::Chain chain_; 
		std::vector<size_t> joint_indeces_;
		std::vector<std::string> joint_names_;		
		bool once_;
		ros::Subscriber sub_state_;
		ros::Subscriber sub_goal_;
		ros::Publisher pub_;
	//	ros::Timer timer_;	
		sensor_msgs::JointState last_joint_state_;
		KDL::JntArray q_out_;
		KDL::Frame goal_frame_;
	};
}

#endif // KDL_TRAINING_KDL_TRANSFORM_HPP
