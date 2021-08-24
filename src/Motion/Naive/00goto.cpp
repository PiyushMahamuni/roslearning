#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include <math.h>

// Constants
const char* NODE_NAME {"goto"};
const char* VEL_TOPIC {"turtle1/cmd_vel"};
const char* POSE_TOPIC {"turtle1/pose"};
const _Float32 LT {0.01};   // linear threshold
const _Float32 pi_by_4{0.785398163};
const _Float32 pi_by_2{pi_by_4 * 2};
const _Float32 pi{pi_by_2 * 2};
const _Float32 pi_2{pi * 2};
const _Float32 AT {(_Float32) (1 * pi_by_4 / 45)};
#define PITOPI 0
#define ZEROTOPI 1
#define PUB_VEL_FREQ 20

// Global
ros::Publisher vel_pub;
ros::Subscriber pose_sub;
ros::Duration* blink;
#define BLINKDUR 1.0/200.0
turtlesim::Pose cpos;
geometry_msgs::Twist vel_msg, stop_msg;
int range_select;
bool to_pub_vel{false}; // controls pub_vel_periodic()
bool pose_updated{false}; // tells if position was updated from last time it was set to false

// callback function for POSE_TOPIC
void pose_callback(const turtlesim::Pose::ConstPtr& msg){
    cpos.x = msg->x;
    cpos.y = msg->y;
    if(range_select == PITOPI)  cpos.theta = msg->theta;
    else cpos.theta = (msg->theta > 0) ? msg->theta : msg->theta + pi_2;
    pose_updated = true;
}

// function will be opened in new thread to periodically publish velocity
void pub_vel_periodic(){
    ros::Rate loop_rate{PUB_VEL_FREQ};
    while(to_pub_vel){
        vel_pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return;
}

// function to stop robot and thread running pub_vel_periodic()
void stop_robot(){
    to_pub_vel = false;
    vel_msg = stop_msg;
    vel_pub.publish(vel_msg);
    ros::spinOnce();
}

// wait for new update on POSE_TOPIC
void wait_for_update(){
    pose_updated = false;
    while(!pose_updated){
        blink->sleep();
        ros::spinOnce();
    }
}

// goes to specified position using proportional controller
bool goto_goal(turtlesim::Pose tpos, _Float32 kpd, _Float32 kpa, bool log = false){
    stop_robot();
    wait_for_update();
    // calculate tpos.theta
    tpos.theta = (_Float32)atan2(tpos.y - cpos.y, tpos.x - cpos.x);
    if(log){
        ROS_INFO("[%s] Command recieved, goto %f %f %f", NODE_NAME,
        tpos.x, tpos.y, tpos.theta);
        std::cout << "Current pos:\n"
                  << "x = " << cpos.x << " y = " << cpos.y << " theta = " << cpos.theta << '\n'
                  << "Target pos:\n"
                  << "x = " << tpos.x << " y = " << tpos.y << " theta = " << tpos.theta << '\n'
                  << "Enter any key to continue, abort to abort: ";
        std::string choice;
        std::getline(std::cin, choice);
        if(choice == "abort"){
            ROS_INFO("[%s] Command aborted by user!", NODE_NAME);
            return false;
        }
    }
    // publish message at 20 Hz per second
    ros::Rate loop_rate{50};
    _Float32 xE, yE, distE, angleE; // proportionality gain distance and angle
    do{
        xE = tpos.x - cpos.x;
        yE = tpos.y - cpos.y;
        distE = (_Float32)sqrt(xE * xE + yE * yE);
        angleE = asin(yE/distE);
        if(tpos.x < cpos.x){
            if(angleE < 0)
                angleE = -pi -angleE;
            else
                angleE = pi - angleE;
        }
        // std::cout << "tpos angle = " << angleE << '\n';
        angleE -= cpos.theta;
        if(angleE < -pi) angleE += pi_2;
        else if(angleE > pi) angleE -= pi_2;
        // std::cout << "angleE = " << angleE << '\n';
        vel_msg.linear.x = kpd * distE;
        vel_msg.angular.z = kpa * angleE;
        vel_pub.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }while((distE > LT || angleE > AT) && ros::ok());

    if(log){
        ROS_INFO("[%s] goto command completed!", NODE_NAME);
        std::cout << "Current location: x = " << cpos.x << " y = " << cpos.y << '\n';
    }
    return true;
}

// setup the node
inline void setup(int argc, char** argv){
    std::cout << "Setting up node\n";
    ros::init(argc, argv, NODE_NAME);
    static ros::NodeHandle node;
    vel_pub = node.advertise<geometry_msgs::Twist>(VEL_TOPIC, 1);
    pose_sub = node.subscribe(POSE_TOPIC, 1, pose_callback);
    blink = new ros::Duration(BLINKDUR);
    std::cout << "Node setup complete!\n";
}

// wrap up node
inline void wrapup(){
    delete blink;
}

// main driver
int main(int argc, char** argv){
    if(argc != 5){
        std::cout << "Usage: " << argv[0] << " x, y, kpd, kpa\n"
                  << "recommended kpd = 0.5, kpa = 2\n";
        return 1;
    }
    setup(argc, argv);
    turtlesim::Pose target;
    target.x = (_Float32)atof(argv[1]);
    target.y = (_Float32)atof(argv[2]);
    _Float32 kpd, kpa;
    kpd = (_Float32)atof(argv[3]);
    kpa = (_Float32)atof(argv[4]);
    goto_goal(target, kpd, kpa, true);
    wrapup();
    return 0;
}