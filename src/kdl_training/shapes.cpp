#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

class Shape
{	
    public:
	Shape(const ros::NodeHandle& nh): nh_(nh)
	{}

	~Shape(){}

	
	visualization_msgs::Marker createBottle(const std::string& frame_id, const std::string& namesp, const size_t& id, const float& red, const float& green, const float& blue)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = frame_id;
		marker.header.stamp = ros::Time();
		marker.ns = namesp;
		marker.id = id;
		marker.type = 10;
		marker.action = visualization_msgs::Marker::ADD;

		marker.pose.position.x = 0.9;
		marker.pose.position.y = -0.4;
		marker.pose.position.z = 0.6;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 0.0;

		marker.scale.x = 1.1;
		marker.scale.y = 1.1;
		marker.scale.z = 1.5;

		marker.color.r = red;
		marker.color.g = green;
		marker.color.b = blue;
		marker.color.a = 1.0;

		marker.mesh_resource = "package://kdl_training/meshes/bottle/bottle.stl";

		marker.lifetime = ros::Duration();

		return marker;
	}


	visualization_msgs::Marker createTable(const std::string& frame_id, const std::string& namesp, const size_t& id)
	{
		visualization_msgs::Marker marker;
		marker.mesh_resource = "package://kdl_training/meshes/chemlab_table/chemlab_table.dae";	
		marker.header.frame_id = frame_id;
		marker.header.stamp = ros::Time();
		marker.ns = namesp;
		marker.id = id;
		marker.type = 10;
		marker.action = visualization_msgs::Marker::ADD;

		marker.pose.position.x = 0.8;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 0.0;

		marker.scale.x = 0.8;
		marker.scale.y = 0.8;
		marker.scale.z = 0.8;

		marker.color.r = 0.38f;
		marker.color.g = 0.48f;
		marker.color.b = 0.55f;
		marker.color.a = 1.0;

		marker.lifetime = ros::Duration();

		return marker;
	}
	
	void start()
	{
		pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
		marker_array_.markers.resize(2);
	//	marker_array_.markers.push_back(createShape("/world", "shapes", 0, 1, 1.0));
		marker_array_.markers.push_back(createBottle("/map", "shapes", 2, 1.0, 0.7, 0.5));
		marker_array_.markers.push_back(createTable("/map", "shapes", 1));

		while (pub_.getNumSubscribers() < 1)
		{
			if (!ros::ok())
			{
				throw std::runtime_error("Problem");
			}
			ROS_WARN_ONCE("Please create a subscriber to the marker");
			sleep(1);
		}
		pub_.publish(marker_array_);
		ROS_INFO("Published!");
	}

    private:
	ros::NodeHandle nh_;
	ros::Publisher pub_;
	visualization_msgs::MarkerArray marker_array_;
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "shapes");
	ros::NodeHandle nh("~");

	Shape shape(nh);
	
	try
   	{
		shape.start();
   	}
   	catch(const std::exception& e)
   	{
   		ROS_ERROR("%s", e.what());
   		return 0;
   	}
	ros::spinOnce();

	return 0;
}
