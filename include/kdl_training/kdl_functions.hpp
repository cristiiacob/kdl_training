#ifndef KDL_TRAINING_KDL_FUNCTIONS_HPP
#define KDL_TRAINING_KDL_FUNCTIONS_HPP

#include <sensor_msgs/JointState.h>
#include <kdl_conversions/kdl_msg.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

namespace kdl_training
{
	// TODO: Add better exception handling

	inline KDL::JntArray toKDL(const sensor_msgs::JointState& msg, const std::vector<size_t>& indeces)
	{
		KDL::JntArray result(indeces.size());
	  	
		for(int i = 0; i < indeces.size(); ++i)
		{
			if (indeces[i] > msg.position.size())
  				throw std::runtime_error("Received index bigger than size of message.");
			result(i) = msg.position[indeces[i]];
	  	}
	
	  	return result;
	}

	inline KDL::Frame calculateFK(const std::shared_ptr<KDL::ChainFkSolverPos_recursive>& fk_solver, const KDL::JntArray& joints)
	{	
		KDL::Frame result;
		fk_solver->JntToCart(joints, result);
		
		return result;
	}

	inline KDL::JntArray calculateIK(const std::shared_ptr<KDL::ChainIkSolverPos_LMA>& ik_solver, const KDL::Frame& goal_frame, const KDL::JntArray& joints_in)
	{	
		KDL::JntArray joints_out;
		ik_solver->CartToJnt(joints_in, goal_frame, joints_out);
		
		return joints_out;
	}

        inline std::vector<std::string> getJointNames(const KDL::Chain& chain)
		{
			std::vector<std::string> joint_names;
			for(size_t i = 0; i < chain.getNrOfSegments(); ++i)
			{	
				KDL::Segment segment = chain.getSegment(i);

				if(segment.getJoint().getTypeName().compare("None"))
				{
					joint_names.push_back(segment.getJoint().getName());	
				}				
			}	
			return joint_names;
		}

	inline std::map<std::string, size_t> createIndexMap(const sensor_msgs::JointState::ConstPtr& msg, const std::vector<std::string>& joint_names)
		{
			std::map<std::string, size_t> joint_index_map;
			for(int i=0; i<msg->name.size(); ++i)
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
					throw std::runtime_error("Could not find joint");
			}		
			return joint_indeces;
		}
}

#endif // KDL_TRAINING_KDL_FUNCTIONS_HPP
