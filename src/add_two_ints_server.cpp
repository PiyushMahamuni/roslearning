#include "ros/ros.h"
//#include "<package_name>/<service_name>.h"
#include "roslearning/AddTwoInts.h"

// Constants
const char *NODE_NAME {"add_two_ints_server_cpp"};
const char *SERVICE_NAME {"add_two_ints"};

// Globals
ros::ServiceServer service;

// Handler function for requests on SERVICE_NAME
bool handle_add(roslearning::AddTwoInts::Request& req, roslearning::AddTwoInts::Response& res){
	res.sum = req.a + req.b;	// Doing only this is enough
	ROS_INFO("[%s] Request: x = %ld, y = %ld", NODE_NAME, (long int)req.a, (long int)req.b);
	ROS_INFO("[%s] Sending back response: [%ld]", NODE_NAME, (long int) res.sum);
	return true;
}
// Service handlers in cpp have two reference parameters as opposed to only one in python


inline void setup_node(int argc, char** argv){
	// initialize node
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle node;
	service = node.advertiseService(SERVICE_NAME, handle_add);
	// to create ServiceServer object, you need to pass two arguments, service name and handler function address
	ROS_INFO("[%s] Ready to add ints", NODE_NAME);
}

int main(int argc, char** argv){
	setup_node(argc, argv);
	ros::spin();
	return 0;
}
