#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

class Shape
{	
    public:
	Shape(const ros::NodeHandle& nh): nh_(nh)
	{}

	~Shape(){}

	
	visualization_msgs::Marker createShape(const std::string& frame_id, const std::string& namesp, const size_t& id, const size_t& shape, const float& x_pos)
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = frame_id;
		marker.header.stamp = ros::Time();
		marker.ns = namesp;
		marker.id = id;
		marker.type = shape;
		marker.action = visualization_msgs::Marker::ADD;

		marker.pose.position.x = x_pos;
		marker.pose.position.y = 1;
		marker.pose.position.z = 1;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 0.0;

		marker.scale.x = 0.5;
		marker.scale.y = 0.5;
		marker.scale.z = 0.5;

		marker.color.r = 0.0f;
		marker.color.g = 1.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;

		if(shape == 10)
			marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

		marker.lifetime = ros::Duration();

		return marker;
	}

	void start()
	{
		pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1);
		marker_array_.markers.resize(2);
		marker_array_.markers.push_back(createShape("/my_frame", "shapes", 0, 1, 1.0));
		marker_array_.markers.push_back(createShape("/my_frame", "shapes", 1, 2, 2.0));
		marker_array_.markers.push_back(createShape("/my_frame", "shapes", 2, 10, 3.0));

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