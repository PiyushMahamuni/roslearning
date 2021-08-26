#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <math.h>

// CONSTANTS
const char* NODE_NAME{"pLinController"};
const char* VEL_TOPIC{"turtle1/cmd_vel"};
const char* POSE_TOPIC{"turtle1/pose"};
const _Float32 LT {0.01}; // linar tolerance
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
ros::Rate* blink{nullptr};   // blinking (short idle) duration

// setup this node
inline void setup(int argc, char** argv);
// callback function for pose_sub
void pose_callback(const turtlesim::Pose::ConstPtr& msg);
// function implementing proportional linear controller
void pLineController(_Float32 x, _Float32 kpd, _Float32 m, bool log = false);
// stops the turtlebot
void stop_robot();
// wait for pose updated
void waitPoseUpdate();

int main(int argc, char** argv){
    if(argc != 4){
        std::cerr << "Usage: " << argv[0] << " [x, m, kpd]\n";
        return 1;
    }
    setup(argc, argv);
    pLineController((_Float32)atof(argv[1]), (_Float32)atof(argv[3]), (_Float32)atof(argv[2]), false);
    return 0;
}

// setup this node
inline void setup(int argc, char** argv){
    // setting up globals
    vel_msg.linear.x = vel_msg.linear.y = vel_msg.linear.z =
    vel_msg.angular.x = vel_msg.angular.y = vel_msg.angular.z = 0;
    stop_msg = vel_msg;
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle node;
    // initialize POSE_TOPIC subscriber
    pose_sub = node.subscribe(POSE_TOPIC, 1, pose_callback);
    // initialize VEL_TOPIC publisher
    vel_pub = node.advertise<geometry_msgs::Twist>(VEL_TOPIC, 1);
    // initialize blink
    blink = new ros::Rate(blinkFreq);
    return;
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
    waitPoseUpdate();
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