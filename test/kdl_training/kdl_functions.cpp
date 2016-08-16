#include <gtest/gtest.h>
#include <kdl_training/kdl_functions.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <ros/ros.h>

using namespace kdl_training;

class TransformTest : public ::testing::Test
{
	protected:
	    virtual void SetUp()
	    {
		msg.name.push_back("torso_lift_joint");
		msg.name.push_back("r_shoulder_pan_joint");
		msg.name.push_back("r_shoulder_lift_joint");
		msg.name.push_back("r_upper_arm_roll_joint");
		msg.name.push_back("r_elbow_flex_joint");
		msg.name.push_back("r_forearm_flex_joint");
		msg.name.push_back("r_wrist_flex_joint");
		msg.name.push_back("r_wrist_roll_joint");
		indeces.push_back(0);
		indeces.push_back(2);
		indeces.push_back(3);
		indeces.push_back(1);
		indeces.push_back(5);
		indeces.push_back(4);
		indeces.push_back(6);
		indeces.push_back(7);
		
		for (size_t i = 0; i < 8; i++)
			msg.position.push_back(0.0);	

		ASSERT_TRUE(urdf.initFile("pr2.urdf"));
		ASSERT_TRUE(kdl_parser::treeFromUrdfModel(urdf, tree));	
		ASSERT_TRUE(tree.getChain("base_link", "r_gripper_tool_frame", chain));

		for(size_t i = 0; i < chain.getNrOfSegments(); i++)
		{	
			KDL::Segment segment = chain.getSegment(i);
			if(segment.getJoint().getTypeName().compare("None"))	
				joint_names.push_back(segment.getJoint().getName());	
		}
	    }

	    virtual void TearDown()
	    {
	      
	    }

		std::vector<std::string> joint_names;
   	        std::vector<size_t> indeces;
	        sensor_msgs::JointState msg;	     
	        urdf::Model urdf;
		KDL::Tree tree;
		KDL::Chain chain;
};

TEST_F(TransformTest, vectorTest)
{	
	ASSERT_EQ(8, toKDL(msg, indeces).rows());

}

TEST_F(TransformTest, nameTest)
{	
	for (size_t i = 0; i < 8; i++)
		ASSERT_STREQ(msg.name[2].c_str(), joint_names[2].c_str());	
}	
