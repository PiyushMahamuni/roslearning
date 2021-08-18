#include "ros/ros.h"
#include "std_msgs/String.h"

// CONSTANTS
const char *NODE_NAME {"listener_cpp"};
const char *TOPIC_NAME {"chat"};

// GLOBALS
ros::Subscriber sub;

// chat subscriber callback
void callback(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("[%s] I heard [%s]", NODE_NAME, msg->data.c_str());
    return;
}

// setting up this node
inline void setup(int argc, char **argv){
    ros::init(argc, argv, NODE_NAME);
    static ros::NodeHandle node;
    // static cause it's important that NodeHandle ojbect lives for as long as program is running

    // setting up subscriber
    sub = node.subscribe(TOPIC_NAME, 10, callback);
    return;
}

// main driver
int main(int argc, char **argv){
    setup(argc, argv);
    ros::spin();    // keep the program from exiting unless terminated by user
    return 0;
}