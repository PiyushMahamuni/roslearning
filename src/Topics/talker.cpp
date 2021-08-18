#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

// CONSTANTS
const char *NODE_NAME{"talker_cpp"};
const char *TOPIC_NAME{"chat"};
const float pub_frequency{1};

// Globals
ros::Publisher pub;
ros::Timer pub_timer;

// pub_timer callback function
void publish_message(const ros::TimerEvent& event){
    static unsigned int count {0};
    static std_msgs::String msg;
    std::stringstream ss;
    ss << "[" << count << "] Hello World!";
    msg.data =ss.str();
    pub.publish(msg);
    count++;
    ROS_INFO("[%s] I published [%s]", NODE_NAME, msg.data.c_str());
    return;
}

// setting up node
inline void setup(int argc, char **argv){
    ros::init(argc, argv, NODE_NAME);
    static ros::NodeHandle node;
    // static cause, it's important that NodeHandle obj has the life of program, otherwise the program
    // terminates prematurely without any error
    pub = node.advertise<std_msgs::String>(TOPIC_NAME, 10);
    pub_timer = node.createTimer(ros::Duration(1/pub_frequency), publish_message);
    return;
}

// doing all things necessary before terminating
inline void packup(){
    // stop all timers
    pub_timer.stop();
}

// main driver
int main(int argc, char **argv){
    // setup the node
    setup(argc, argv);
    // keep the prgoram exiting unless terminated by user
    ros::spin();
    // packing up to terminate properly
    packup();
    return 0;
}