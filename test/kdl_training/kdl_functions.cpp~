#include <gtest/gtest.h>
#include <kdl_training/kdl_functions.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

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
		
		joint_initial.resize(8);
		joint_test.resize(8);

		joint_test(0) = 0.228028;
		joint_test(1) = -0.967708;
		joint_test(2) = 0.54159;
		joint_test(3) = -2.36991;
		joint_test(4) = -1.15363;
		joint_test(5) = 6.29081;
		joint_test(6) = -1.17101;
		joint_test(7) = 0.753118;
	
		for (size_t i = 0; i < q_in.rows(); ++i)
			joint_initial(i) = 0.0;

	
		ik_solver = std::make_shared<KDL::ChainIkSolverPos_LMA>(KDL::ChainIkSolverPos_LMA(chain));	
	    	pose_msg.position.x = 0.360;
		pose_msg.position.y = -0.260;
		pose_msg.position.z = 0.498;
		pose_msg.orientation.x = 0.005;
		pose_msg.orientation.y = 0.072;
		pose_msg.orientation.z = 0.725;
		pose_msg.orientation.w = 0.685;

		tf::poseMsgToKDL(pose_msg, goal_frame);	
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

		KDL::JntArray joint_out;	//joint positions form solver
		KDL::JntArray joint_test;   	//joint test positions from georg
		KDL::JntArray joint_initial;	//initial positions, set to 0
		// or set to the same position? 
		KDL::Frame goal_frame;
		std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver;	
		geometry_msgs::Pose pose_msg;
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

TEST_F(TransformTest, ikTest)
{	
	joint_out = calculateIK(ik_solver, goal_frame, q_in);
	//for (size_t i = 0; i < q_out.rows(); ++i)
		ASSERT_EQ(joint_test(0), joint_out(0));	
		ASSERT_EQ(joint_test(1), joint_out(1));
		ASSERT_EQ(joint_test(2), joint_out(2));
		ASSERT_EQ(joint_test(3), joint_out(3));
		ASSERT_EQ(joint_test(4), joint_out(4));
		ASSERT_EQ(joint_test(5), joint_out(5));
		ASSERT_EQ(joint_test(6), joint_out(6));
		ASSERT_EQ(joint_test(7), joint_out(7));
}
