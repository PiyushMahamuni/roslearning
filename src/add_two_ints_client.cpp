#include "ros/ros.h"
#include "roslearning/AddTwoInts.h"
#include <cstdlib>

// Constants
const char *NODE_NAME{"add_two_ints_client_cpp"};
const char *SERVICE_NAME{"add_two_ints"};

// Globals
ros::ServiceClient client;

void setup_node(int argc, char** argv){
	// initialize node
	ros::init(argc, argv, NODE_NAME);
	static ros::NodeHandle node;
	// STATIC IS IMPORTANT, THE NODE OBJECT SHOULD HAVE THE LIFE SAME AS PROGRAM, IF IT GETS DESTROYED BEFORE THAT, THE
	// PROGRAM WILL BE TERMINATED PREMATURELY, WITHOUT ANY ERRORS.
	// YOU COULD USE A GLOBAL POINTER AND DMA INSTEAD TOO, HERE WE DON'T REALLY NEED TO ACCESS node BEYOND THIS FUNCTION
	// SO STATIC WOULD DO JUST FINE
	
	client = node.serviceClient<roslearning::AddTwoInts>(SERVICE_NAME);
	// To create ros::ServiceClient objects, you need a template parameter telling the service, and single string argument
	// telling service name
}

int main(int argc, char** argv){
	if(argc != 3){
		std::cout << "Usage: " << argv[0] << " X Y" << std::endl;
		return 1;
	}
	
	setup_node(argc, argv);
	
	// create a service msg object to hold the values
	roslearning::AddTwoInts srv;
	srv.request.a = atoll(argv[1]); // atoll convents const char* string to long long int
	srv.request.b = atoll(argv[2]);
	
	// bool ros::ServiceClient::call(<package_name>::<Service Type>& msg_type_obj)
	if(client.call(srv)){
		ROS_INFO("[%s] Response recieved, sum = [%ld]", NODE_NAME, (long int)srv.response.sum);
	}
	else{
		ROS_INFO("[%s] Failed to recieve response of %s service call", NODE_NAME, SERVICE_NAME);
		return 1;
	}
	return 0;
}
