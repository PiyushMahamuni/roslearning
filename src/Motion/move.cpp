// NOT WORKING, need to search on threads and timers

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
ros::SteadyTimer vel_pub_timer;
turtlesim::Pose cpos, tpos;
geometry_msgs::Twist vel_msg, stop_msg;
ros::Duration *blink;
bool updated_pose {false};

// pose_sub callback functin
void pose_sub_callback(const turtlesim::Pose::ConstPtr& msg){
    cpos.x = msg->x;
    cpos.y = msg->y;
    cpos.theta = msg->theta;
    // std::cout << "Received message!" << std::endl; // Debug
    updated_pose = true;
}

// vel_pub_timer callback function
void pub_vel(const ros::SteadyTimerEvent& event){
    vel_pub.publish(vel_msg);
    ros::spinOnce();
    std::cout << "Periodically publishing vel!" << std::endl; // debugging
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
    vel_pub_timer = node.createSteadyTimer(ros::WallDuration(1.0/VEL_PUB_FREQ), pub_vel, false, false);
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
	// avoid conflicting speed and distance values
	speed = (distance>0)?abs(speed):-abs(speed);
    updated_pose = false;
    while(!updated_pose){
        ros::spinOnce();    // update cpos
        blink->sleep();
    }
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
        std::getline(std::cin, choice);
        if(choice == "abort"){
            ROS_INFO("[%s] User aborted the operation!", NODE_NAME);
            return false;
        }
    }

    ros::Duration* sleep {nullptr}; // always initialize pointers that you delete to nullptr
    // calculate sleep duration
    bool to_sleep {abs(distance) > LT};
	
	
	// BUG - THE PROGRAM SLEEPS HERE FOR INTENDED TIME, BUT THE VEL_PUB_TIMER ALSO BECOMES INEFFECTIVE
	// AND STOPS PERIODICALLY PUBLISHING VELOCITIES, WITHOUT WHICH THE ROBOT ALSO STOPS AFTER SOME TIME
	
	
	if(to_sleep){
        float temp = (abs(distance) - LT) / abs(speed);
        if(log) std::cout << "Sleeping time: " << temp << std::endl;
        sleep = new ros::Duration{temp};
    }
    // start moving robot
    vel_msg.linear.x = speed;
    vel_pub.publish(vel_msg);
    ros::spinOnce();
    //std::cout << "published message!" << std::endl;
    // start the vel_pub_timer
    vel_pub_timer.start();
    if(to_sleep){
        if(log) std::cout << "Sleeping..." << std::endl;
        sleep->sleep();
        if(log) std::cout << "Awake..." << std::endl;
    }
    // slow down if needed
    if(vel_msg.linear.x > approach_speed){
    	vel_msg.linear.x = approach_speed;
    	vel_pub.publish(vel_msg);
    	ros::spinOnce();
    }

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
        if(tpos.x > cpos.x){
            while(tpos.x > cpos.x){
                ros::spinOnce();
                blink->sleep();
            }
        } else {
            while(tpos.x < cpos.x){
                ros::spinOnce();
                blink->sleep();
            }
        }
    }
    stop_robot();
    vel_pub_timer.stop();
    if(log){
        ROS_INFO("[%s] Command completed!", NODE_NAME);
        std::cout << "Current position: x = " << cpos.x << " y = " << cpos.y << std::endl;
    }
    delete sleep;
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
