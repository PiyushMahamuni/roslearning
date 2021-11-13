#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <math.h>

// CONSTANTS
const char* NODE_NAME{"pLinController"};
const char* VEL_TOPIC{"/turtle1/cmd_vel"};
const char* POSE_TOPIC{"/turtle1/pose"};
// can't use relative topic name since we will be using a private nodehandle to be able to access
// private parameters from parameter server.
const _Float32 LT {0.01}; // linar threshold
const float blinkFreq {100.0};
const float controllerFreq{100.0};

// GLOBALS
ros::Publisher vel_pub;
ros::Subscriber pose_sub;
turtlesim::Pose cpos; // current position
geometry_msgs::Twist vel_msg;   // velocity message to be published on VEL_TOPIC
geometry_msgs::Twist stop_msg;  // velocity message having all fields set to zero
bool poseUpdated {false};   // tells whether position is updated since the last time this variable
// was set to false
ros::Rate* blink{nullptr};   // blinking (short/idle) duration
ros::NodeHandle* node{nullptr};

// setup this node
inline void setup(int argc, char** argv);
// wrap up the node
inline void wrapup();
// callback function for pose_sub
void pose_callback(const turtlesim::Pose::ConstPtr& msg);
// function implementing proportional linear controller
void pLineController(_Float32 x, _Float32 kpd, _Float32 m, bool log = false);
// stops the turtlebot
void stop_robot();
// wait for pose updated
void waitPoseUpdate();

int main(int argc, char** argv){
    setup(argc, argv);
    float x, kpd, m;
    bool success {true};
    success = success && node->getParam("x0", x);
    if (success)
        success = success && node->getParam("kpd", kpd);
    if (success)
    success = success && node->getParam("m", m);
    if (success)
        pLineController(x, kpd, m, true);
    else
        ROS_INFO("[%s] Failed to retrieve the parameters!", NODE_NAME);
        return 1;
    ROS_INFO("[%s] Retrieved parameters! x= %f, kpd= %f, m= %f", NODE_NAME, x, kpd, m);
    wrapup();
    return 0;
}

// setup this node
inline void setup(int argc, char** argv){
    // setting up globals
    vel_msg.linear.x = vel_msg.linear.y = vel_msg.linear.z =
    vel_msg.angular.x = vel_msg.angular.y = vel_msg.angular.z = 0;
    stop_msg = vel_msg;
    ros::init(argc, argv, NODE_NAME);
    node = new ros::NodeHandle("~");

    // initialize POSE_TOPIC subscriber
    pose_sub = node->subscribe(POSE_TOPIC, 1, pose_callback);
    // initialize VEL_TOPIC publisher
    vel_pub = node->advertise<geometry_msgs::Twist>(VEL_TOPIC, 1);
    // initialize blink
    blink = new ros::Rate(blinkFreq);
    return;
}

// wrapup this node
inline void wrapup(){
    delete blink;
    delete node;
}
// callback function for pose_sub
void pose_callback(const turtlesim::Pose::ConstPtr& msg){
    cpos.x = msg->x;
    cpos.y = msg->y;
    cpos.theta = msg->theta;
    poseUpdated = true;
    return;
}

// function implementing proportional linear controller
// log is default parameter having false value
void pLineController(_Float32 x, _Float32 kpd, _Float32 m, bool log){
    // this controls the force acted on robot, not velocity of robot
    stop_robot();
    
    try
    {
        waitPoseUpdate();
    }
    catch(const ros::Exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    ros::Rate ControllerRate {controllerFreq};
    _Float32 loop_dur {(_Float32)ControllerRate.expectedCycleTime().toSec()};
    x = abs(x);
    _Float32 dx {x - cpos.x}, pv{0}, cv{}, K{kpd*loop_dur/(2*m)}; // prev velocity, current velocity
    do{
        cv = K * dx; // this is avg velocity
        vel_msg.linear.x += (pv + cv);
        pv = cv;
        vel_pub.publish(vel_msg);
        blink->sleep();
        ros::spinOnce();
        dx = x - cpos.x;
    }while(ros::ok());
    return;
}

// stops the robot
void stop_robot(){
    vel_msg = stop_msg;
    vel_pub.publish(vel_msg);
    ros::spinOnce();
}

// wait for pose updated
void waitPoseUpdate(){
    poseUpdated = false;
    while(!poseUpdated){
        blink->sleep();
        ros::spinOnce();
    }
}