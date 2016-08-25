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
		
		msg.position.push_back(0.228028);
		msg.position.push_back(-0.967708);
		msg.position.push_back(0.54159);
		msg.position.push_back(-2.36991);
		msg.position.push_back(-1.15363);
		msg.position.push_back(6.29081);
		msg.position.push_back(-1.17101);
		msg.position.push_back(0.753118);

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

	//	for (size_t i = 0; i < joint_initial.rows(); ++i)
	//		joint_initial(i) = 0.0;
	
		joint_initial(0) = 0.228028;
		joint_initial(1) = -0.967708;
		joint_initial(2) = 0.54159;
		joint_initial(3) = -2.36991;
		joint_initial(4) = -1.15363;
		joint_initial(5) = 6.29081;
		joint_initial(6) = -1.17101;
		joint_initial(7) = 0.753118;
	
		ik_solver = std::make_shared<KDL::ChainIkSolverPos_LMA>(KDL::ChainIkSolverPos_LMA(chain));	
		fk_solver = std::make_shared<KDL::ChainFkSolverPos_recursive>(KDL::ChainFkSolverPos_recursive(chain));

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
		KDL::JntArray joint_initial;	//initial positions, set to 0
		// or set to the same position? 
		KDL::Frame goal_frame;  // goal data from georg --> cartesian
		KDL::Frame goal_frame_fk; // fk(ik(goal data from georg)) --> cartesian
		std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver;	
		std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver;
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

TEST_F(TransformTest, fkTest)
{	
	joint_out = calculateIK(ik_solver, goal_frame, joint_initial);
	goal_frame_fk = calculateFK(fk_solver, joint_out);
	EXPECT_TRUE(Equal(goal_frame, goal_frame_fk, 0.01));
	
	for(size_t i =0; i < 3; ++i)
	{
		EXPECT_EQ(goal_frame.p[i], goal_frame_fk.p[i]);
		for(size_t j = 0; j < 3; ++j)
			EXPECT_EQ(goal_frame.M(i,j), goal_frame_fk.M(i,j));	
	}
	
}
