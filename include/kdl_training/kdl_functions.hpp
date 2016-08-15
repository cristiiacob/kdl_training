#ifndef KDL_TRAINING_KDL_FUNCTIONS_HPP
#define KDL_TRAINING_KDL_FUNCTIONS_HPP

#include <sensor_msgs/JointState.h>
#include <kdl_conversions/kdl_msg.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>

namespace kdl_training
{
	// TODO: separate this in hpp-file and cpp-file
	// TODO: afterwards, write unit-tests for this
	inline KDL::JntArray toKDL(const sensor_msgs::JointState& msg, const std::vector<size_t>& indeces)
	{
		KDL::JntArray result = KDL::JntArray(indeces.size());
	  	
		for(int i = 0; i < indeces.size(); i++)
		{
			result(i) = msg.position[indeces[i]];
	  	}
	
	  	return result;
	}

	inline KDL::Frame calculateFK(const KDL::Chain& chain, const KDL::JntArray& joints)
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

        inline std::vector<std::string> getJointNames(const KDL::Chain& chain)
		{
			ROS_INFO("Nr of joints: %d",chain.getNrOfJoints());
			ROS_INFO("Total nr of segments: %d",chain.getNrOfSegments());
		
			std::vector<std::string> joint_names;
			for(size_t i = 0; i < chain.getNrOfSegments(); i++)
			{	
				KDL::Segment segment = chain.getSegment(i);

				if(segment.getJoint().getTypeName().compare("None"))
				{
					ROS_INFO("%s",segment.getJoint().getName().c_str());
					joint_names.push_back(segment.getJoint().getName());	
				}				
			}	
			return joint_names;
		}

	inline std::map<std::string, size_t> createIndexMap(const sensor_msgs::JointState::ConstPtr& msg, const std::vector<std::string>& joint_names)
		{
			std::map<std::string, size_t> joint_index_map;
			for(int i=0; i<msg->name.size(); i++)
			{
				joint_index_map[msg->name[i]] = i;
			}
			return joint_index_map;
		}

	inline std::vector<size_t> createJointIndices(const std::map<std::string, size_t>& joint_index_map, const std::vector<std::string>& joint_names)
		{
			std::vector<size_t> joint_indeces;
			for(size_t i=0; i<joint_names.size(); ++i)
			{
	  			std::map<std::string,size_t>::const_iterator it = joint_index_map.find(joint_names[i]);
	 		        if (it != joint_index_map.end())
	   				joint_indeces.push_back(it->second);
	  			else
	    				ROS_ERROR("Could not find joint '%s'", joint_names[i].c_str());
			}		
			return joint_indeces;
		}

}

#endif // KDL_TRAINING_KDL_FUNCTIONS_HPP
