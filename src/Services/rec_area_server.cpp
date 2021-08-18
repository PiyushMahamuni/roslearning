#include "ros/ros.h"
//#include "<package_name>/<service_name>.h"
#include "roslearning/rec_area.h"

// Constants
const char *NODE_NAME {"rec_area_server_cpp"};
const char *SERVICE_NAME {"rec_area"};

// Globals
ros::ServiceServer service;

// Handler function for requests on SERVICE_NAME
bool handle_rec_area(roslearning::rec_area::Request& req, roslearning::rec_area::Response& res){
	res.area = req.width * req.height;	// Doing only this is enough
	ROS_INFO("[%s] Request: width = %lf, height = %lf", NODE_NAME, req.width, req.height);
	ROS_INFO("[%s] Sending back response: [%lf]", NODE_NAME, res.area);
	return true;
}
// Service handlers in cpp have two reference parameters as opposed to only one in python

// if not inline, make sure that NodeHandle object is static
inline void setup_node(int argc, char** argv){
	// initialize node
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle node;
	service = node.advertiseService(SERVICE_NAME, handle_rec_area);
	// to create ServiceServer object, you need to pass two arguments, service name and handler function address
	ROS_INFO("[%s] Ready to serve", NODE_NAME);
}

int main(int argc, char** argv){
	setup_node(argc, argv);
	ros::spin();
	return 0;
}
