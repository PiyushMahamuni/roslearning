#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <math.h>

// CONSTANTS
const char* NODE_NAME{"iLinController"};
const char* VEL_TOPIC{"/turtle1/cmd_vel"};
const char* POSE_TOPIC{"/turtle1/pose"};
// NOTE- with private nodehandle, topics, services with relative name are advertised in the namespace of
// the nodehanlde, therefore, we need to use the absolute names of the topics here.
const _Float32 LT {0.01}; // linear threshold
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
ros::NodeHandle* node{nullptr};

// setup this node
inline void setup(int argc, char** argv);
// wrap up this node
inline void wrapup();
// callback function for pose_sub
void pose_callback(const turtlesim::Pose::ConstPtr& msg);
// function implementing integral linear controller
void iLineController(_Float32 x, _Float32 kid, _Float32 m, bool log = false);
// stops the turtlebot
void stop_robot();
// wait for pose updated
void waitPoseUpdate();


int main(int argc, char** argv){
    std::cout << "Info -------------------------" << std::endl
    << "This program works along with turtlesim_node, emulating that robot has\n"
    << "Given mass `m` it implements a integral cotroller whose output is a `force`\n"
    << "Which is imparted on body of robot to make it move\n"
    << "Note, this program doesn't take in account the angle of robot and assumes it is at 0 rad\n";
    setup(argc, argv);
    float x0, m, kid;
    bool success{true};
    success = success && node->getParam("x0", x0);
    if(success)
        success = success && node->getParam("m", m);
    if(success)
        success = success && node->getParam("kid", kid);
    if(!success){
        ROS_INFO("[%s] Failed to retrieve parameters!\n", NODE_NAME);
        ros::shutdown();
        return 1;
    }

    iLineController(x0, kid, m, true);
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
    // using "~" arg to declare that this will be a private nodehandle and with that, 
    // we can access private parameters with NodeHandle::getPrama()

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

// function implementing integral linear controller
// log is default parameter having false value
void iLineController(_Float32 x, _Float32 kid, _Float32 m, bool log){
    // this controls the force acted on robot, not velocity of robot
    stop_robot();
    waitPoseUpdate();
    ros::Rate ControllerRate {controllerFreq};
    _Float32 loop_dur {(_Float32)ControllerRate.expectedCycleTime().toSec()};
    x = abs(x);

    // playground
    /*  cforce = 0
        pv = 0
        loop ----
        dforce = kid * dx
        cforce += dforce;
        cv = cforce / (2 * m) * loop_dur
        vel_msg.linear.x = (pv + cv)
        pv = cv

    */
    // ~playground
    _Float32 dx {x - cpos.x}, pv{0}, cv{}, 
             force{}, K{loop_dur/(2*m)}, dforce{}; // prev change in velocity, change in velocity
    do{
        dforce += kid * dx;
        force += dforce;
        cv = force * K;
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
        try{
        blink->sleep();
        ros::spinOnce();
        }
        catch(ros::Exception& e){
            ROS_INFO("[%s] Exception: %s", NODE_NAME, e.what());
        }
    }
}