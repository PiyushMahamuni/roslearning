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
// function implementing proportional-derivative linear controller
void pdLineController(_Float32 x, _Float32 kpd, _Float32 kdd, _Float32 m, bool log = false);
// stops the turtlebot
void stop_robot();
// wait for pose updated
void waitPoseUpdate();

int main(int argc, char** argv){
    if(argc != 5){
        std::cerr << "Usage: " << argv[0] << " [x, m, kpd, kdd]\n";
        std::cout << "Info -------------------------" << std::endl
        << "This program works along with turtlesim_node, emulating that robot has\n"
        << "Given mass `m` it implements a proportional-derrivative cotroller whose output is a `force`\n"
        << "Which is imparted on body of robot to make it move\n"
        << "Note, this program doesn't take in account the angle of robot and assumes it is at 0 rad\n";
        
        return 1;
    }
    setup(argc, argv);
    pdLineController((_Float32)atof(argv[1]), (_Float32)atof(argv[3]), 
                     (_Float32)atof(argv[4]), (_Float32)atof(argv[2]), true);
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

// function implementing proportional-derivative linear controller
// log is default parameter having false value
void pdLineController(_Float32 x0, _Float32 kpd, _Float32 kdd, _Float32 m, bool log){
    // this controls the force acted on robot, not velocity of robot
    stop_robot();
    waitPoseUpdate();
    if(log){
        ROS_INFO("[%s] Command recieved for Linear PD control");
        std::cout << "m = " << m << " x0 = " << x0 << " kpd = " << kpd << " kdd = " << kdd << '\n'
                  << "Enter any key to continue, abort to abort: ";
        std::string choice;
        std::getline(std::cin, choice);
        if(choice == "abort"){
            ROS_INFO("[%s] Command aborted by user!");
            return;
        }
        std::cout << "Enter ctrl+c to stop the controller!\n";
    }
    ros::Rate ControllerRate {controllerFreq};
    _Float32 loop_dur {(_Float32)ControllerRate.expectedCycleTime().toSec()};
    x0 = abs(x0); // there aren't any -ve x values on screen of turtlesim simulator
    kdd /= loop_dur;
    _Float32 _s2 {x0 - cpos.x}, _s1 {_s2}, pv{0}, cv{}, K{loop_dur/(2*m)},
    force{}; // prev velocity, current velocity

    // playground
    /*
        Let s be the relative displacement of robot w.r.t. target/desired location/point
        let x0 be the desired value, x be the current position
        relative displacement s = x - x0 = error
        Proportional controller cats like spring,
        restoring force fp = -Kp * s; // NEGATIVELY PROPORTIONAL SINCE IT OPPOSES DISPLACEMENT
        fp = Kp * (x0 - x);
        // restoring force acts in opposite direction to that of displacement
        let s1 be a displacement at time t1, s2 at time t2=t1+dt
        ds = s2 - s1
        avg velocity v = ds/dt = rate of change of error
        Derrivative controller acts like dampner, opposing velocity
        fd = -Kd * v;
        fd = Kd (s1 - s2)/ dt;
        if dt is const, Kd = Kd2
        fd = Kd2 (s1 - s2);
        Use suffix 2 for all current values, suffix 1 for all previous values
        let -s be _s = x0 - x
        f = fp + fd
          = -Kp * s2 + (-Kd v2)
          = Kp * (_s2) + Kd2 (s1 - s2)
          = Kp * (_s2) + kd2 (_s2 - _s1)
    */
    // ~playground
    do{
        force = kpd * _s2 + kdd * (_s2 - _s1);
        cv = force * K;
        vel_msg.linear.x += (pv + cv);
        pv = cv;
        _s1 = _s2;
        vel_pub.publish(vel_msg);
        blink->sleep();
        ros::spinOnce();
        _s2 = x0 - cpos.x;
    }while(ros::ok());
    if(log) ROS_INFO("[%s] PD Linear Controller stopped!");
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