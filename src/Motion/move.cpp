#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <math.h>

// CONSTANTS
const char *NODE_NAME {"turtle_control"};
const char *VEL_TOPIC {"turtle1/cmd_vel"};
const char *POSE_TOPIC {"turtle1/pose"};
#define VEL_PUB_FREQ 20.0
const _Float32 pi_by_4 {0.785398163};
const _Float32 LT {0.3};
const _Float32 approach_speed {0.3};
const _Float32 blink_dur {1.0/200};

// GLOBALS
ros::Publisher vel_pub;
ros::Subscriber pose_sub;
ros::Timer vel_pub_timer;
turtlesim::Pose cpos, tpos;
geometry_msgs::Twist vel_msg, stop_msg;
ros::Duration *blink;

// pose_sub callback functin
void pose_sub_callback(const turtlesim::Pose::ConstPtr& msg){
    cpos.x = msg->x;
    cpos.y = msg->y;
    cpos.theta = msg->theta;
    std::cout << "Received message!" << std::endl; // Debug
}

// vel_pub_timer callback function
void pub_vel(const ros::TimerEvent& event){
    vel_pub.publish(vel_msg);
    ros::spinOnce();
}

// function to stop robot
inline void stop_robot(){
    vel_msg = stop_msg;
    vel_pub.publish(vel_msg);
    ros::spinOnce();
}

// setting up this node
inline void setup(int argc, char **argv){
    std::cout << "Setting up node..." << std::endl;
    // Initialize node
    ros::init(argc, argv, NODE_NAME);
    // Initialize node handle
    static ros::NodeHandle node;
    // Initialize pose subscriber
    pose_sub = node.subscribe(POSE_TOPIC, 10, pose_sub_callback);
    // Initialize vel publisher
    vel_pub = node.advertise<geometry_msgs::Twist>(VEL_TOPIC, 1);
    // Initialize vel publisher timer
    vel_pub_timer = node.createTimer(ros::Duration(1.0/VEL_PUB_FREQ), pub_vel, false, false);
    stop_msg.linear.x = 0;
    stop_msg.linear.y = 0;
    stop_msg.linear.z = 0;
    stop_msg.angular.x = 0;
    stop_msg.angular.y = 0;
    stop_msg.angular.z = 0;
    // setup blink duration
    blink = new ros::Duration(blink_dur);
    std::cout << "Setting up node complete..." << std::endl;
}

// wrapping up the node
inline void wrapup(){
    // stop robot
    stop_robot();
    // stop all timers
    // delete any Dynamically Allocated Memory
    vel_pub_timer.stop();
    delete blink;
}

// move function
bool move(_Float32 distance, _Float32 speed, bool log = false){
    ros::spinOnce();    // update cpos
    // calculate tpos
    tpos.x = cpos.x + distance * cos(cpos.theta);
    tpos.y = cpos.y + distance * sin(cpos.theta);
    if(log){
        ROS_INFO("[%s] Command recieved to make the robot move through %f distance with %f speed",
        NODE_NAME, distance, speed);
        std::cout << "Current position: x = " << cpos.x << " y = " << cpos.y << std::endl
                  << "Target position:  x = " << tpos.x << " y = " << tpos.y << std::endl
                  << "Enter any key to continue, Enter abort to abort: ";
        std::string choice;
        std::cin >> choice;
        if(choice == "abort"){
            ROS_INFO("[%s] User aborted the operation!", NODE_NAME);
            return false;
        }
    }

    ros::Duration sleep;
    // calculate sleep duration
    bool to_sleep {abs(distance) > LT};
    if(to_sleep){
        sleep = ros::Duration{(abs(distance) - LT)/speed};
    }
    // start moving robot
    for(int i{}; i<3; i++){
        vel_msg.linear.x = speed;
        vel_pub.publish(vel_msg);
    }
    ros::spinOnce();
    std::cout << "published message!" << std::endl;
    // start the vel_pub_timer
    vel_pub_timer.start();
    if(to_sleep){
        sleep.sleep();
    }
    // slow down if needed
    if(vel_msg.linear.x > approach_speed) vel_msg.linear.x = approach_speed;
    vel_pub.publish(vel_msg);
    ros::spinOnce();

    // Decide whether to use x or y cord
    if(abs(cpos.theta) > pi_by_4 && abs(cpos.theta) < 3 * pi_by_4){
        // use y cord
        if(tpos.y > cpos.y){
            while(tpos.y > cpos.y){
                ros::spinOnce();
                blink->sleep();
            }
        } else {
            while(tpos.y < cpos.y){
                ros::spinOnce();
                blink->sleep();
            }
        }
    } else {
        if(tpos.y > cpos.y){
            while(tpos.y > cpos.y){
                ros::spinOnce();
                blink->sleep();
            }
        } else {
            while(tpos.y < cpos.y){
                ros::spinOnce();
                blink->sleep();
            }
        }
    }
    stop_robot();
    if(log){
        ROS_INFO("[%s] Command completed!", NODE_NAME);
        std::cout << "Current position: x = " << cpos.x << " y = " << cpos.y << std::endl;
    }
    return true;
}

int main(int argc, char **argv){
    if(argc != 3){
        std::cout << "Usage: " << argv[0] << " [distance] [speed]" << std::endl;
        return 1;
    }
    setup(argc, argv);
    move((_Float32)atof(argv[1]), (_Float32)atof(argv[2]), true);
    wrapup();
}