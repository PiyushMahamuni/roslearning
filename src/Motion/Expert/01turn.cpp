// NOT WORKING, need to search on threads and timers

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <math.h>
#include <thread>
#include <chrono>
#include "roslearning/Turn.h"

// CONSTANTS
const char *NODE_NAME{"turn_node"};
const char *VEL_TOPIC{"/turtle1/cmd_vel"};
const char *POSE_TOPIC{"/turtle1/pose"};
const char *TURN_SERVICE{"turn"};

#define VEL_PUB_FREQ 20.0
const _Float32 pi{3.141592654};
const _Float32 pi_by_4{pi / 4};
const _Float32 pi_2{pi * 2};
const _Float32 AT{5 * static_cast<_Float32>(pi/36.0)}; // 5 degrees in radians
const _Float32 LAS {pi};      // limiting angular speed (rad/s)
const _Float32 AAS {static_cast<_Float32>(15 * pi / 180)};
const _Float32 blink_dur{1.0 / 200.0};
int range_select {};
#define PITOPI 0
#define ZEROTO2PI 1

// GLOBALS
ros::Publisher vel_pub;
ros::Subscriber pose_sub;
ros::ServiceServer turn_srvr;
turtlesim::Pose cpos, tpos;
geometry_msgs::Twist vel_cmd, stop_cmd;
ros::Duration *blink;
bool updated_pose{false};
bool to_pub_vel{false};


// PROTOTYPES
// publishes vel_cmd periodically, is meant to run on a seperate thread from main one
void pub_vel_periodic();
// callback function for pose_sub
void pose_sub_callback(const turtlesim::Pose::ConstPtr& msg);
// waits for new update on pose topic from the last time updated_pose global variable was set to false
inline void wait_for_update();
// stops the robot
inline void stop_robot();
// setup this node
inline void setup();
// wrapup this node
inline void wrapup();
// turns the robot through given radians angle with given ang speed (rad/s)
void turn(_Float32 radians, _Float32 speed, bool log = false);
// turn_srvr handler
bool turn_handler(roslearning::Turn::Request& req, roslearning::Turn::Response& res);

// function to call periodically to publish vel_cmd on vel_topic
void pub_vel_periodic()
{
    auto loop_idle_time = std::chrono::microseconds(static_cast<long long int>(1000000 * 1.0 / VEL_PUB_FREQ));
    u_int32_t count{0};
    auto start = std::chrono::high_resolution_clock::now();
    while (to_pub_vel)
    {
        vel_pub.publish(vel_cmd);
        ros::spinOnce();
        count++;
        std::this_thread::sleep_until(start + count * loop_idle_time);
    }
    return;
}

// pose_sub callback functin
void pose_sub_callback(const turtlesim::Pose::ConstPtr &msg)
{
    cpos.x = msg->x;
    cpos.y = msg->y;
    cpos.theta = (range_select == PITOPI)? msg->theta : ((msg->theta < 0) ? (pi_2 + msg->theta) : msg->theta);
    updated_pose = true;
    return;
}

// wait for update on pose topic
inline void wait_for_update(){
    updated_pose = false;
    try{
        while(!updated_pose){
            blink->sleep();
            ros::spinOnce();
        }
    }
    catch(ros::Exception& e){
        ROS_INFO("[%s] Exception: %s", NODE_NAME, e.what());
        ros::shutdown();
    }
    return;
}

// function to stop robot
inline void stop_robot()
{
    to_pub_vel = false;
    vel_cmd = stop_cmd;
    vel_pub.publish(vel_cmd);
    ros::spinOnce();
    return;
}

// setting up this node
inline void setup(int argc, char **argv)
{
    std::cout << "Setting up " << NODE_NAME << " node..." << std::endl;
    // Initialize node
    ros::init(argc, argv, NODE_NAME);
    // Initialize node handle
    static ros::NodeHandle node;
    // Initialize pose subscriber
    pose_sub = node.subscribe(POSE_TOPIC, 10, pose_sub_callback);
    // Initialize vel publisher
    vel_pub = node.advertise<geometry_msgs::Twist>(VEL_TOPIC, 1);
    // Initialize service servers
    turn_srvr = node.advertiseService(TURN_SERVICE, turn_handler);
    stop_cmd.linear.x = 0;
    stop_cmd.linear.y = 0;
    stop_cmd.linear.z = 0;
    stop_cmd.angular.x = 0;
    stop_cmd.angular.y = 0;
    stop_cmd.angular.z = 0;
    // setup blink duration
    blink = new ros::Duration(blink_dur);
    std::cout << "Setting up node complete..." << std::endl;
    return;
}

// wrapping up the node
inline void wrapup()
{
    // stop robot
    stop_robot();
    // stop all timers
    // delete any Dynamically Allocated Memory
    delete blink;
}

// turn_srvr service handler
bool turn_handler(roslearning::Turn::Request& req, roslearning::Turn::Response& res){
    turn(req.radians, req.speed, true);
    return res.success = true;
}

// MOTION -------------------------------------------------------------------------------------------
// turns robot by given amount of radians
void turn(_Float32 radians, _Float32 speed, bool log){
    // make sure that robot has stopped
    stop_robot();

    speed = abs(speed); // make sure to avoid conflicting values
    if(speed == 0){
        ROS_ERROR("[%s] Invalid angular speed! Aborting operation", NODE_NAME);
        return;
    }
    else if(speed > LAS){
        ROS_ERROR("[%s] Angular speed can't be greater than %f rad/s", NODE_NAME, LAS);
        std::cerr << "Speed given: " << speed << std::endl;
        return;
    } else if(radians == 0){
        ROS_INFO("[%s] turn angle is 0!", NODE_NAME);
        return;
    }

    // wait for new updated
    wait_for_update();
    static _Float32 ltt {cpos.theta};
    // using last target theta (ltt) to avoid error accumulation in all turn calls

    // calculate target theta
    tpos.theta = ltt + radians; // use ltt instead of cpos.theta
    while(tpos.theta > pi) tpos.theta -= pi_2;
    while(tpos.theta < -pi) tpos.theta += pi_2;
    // now tpos.theta must be in range from -pi to pi
    ltt = tpos.theta; // next time, ltt should have been achieved but robot is at cpos.theta

    // select which range to use, the radian angle of pi is troublesome in current range as
    // theta value jumps from one end to another, to make it continous around angle pi, we can
    // use range 0 to 2 pi which shifts the troublesome jump point to 0 radians

    if(log){  
        ROS_INFO("[%s] Command recieved to make the robot turn %f radians with %f speed",
        NODE_NAME, radians, speed);
        std::cout << "Current theta: " << cpos.theta << " Target theta: " << tpos.theta << std::endl;
    }

    // avoid conflicting speed and radians values
    speed = (radians > 0)? speed : -speed;

    range_select = (tpos.theta > 3 * pi_by_4 || tpos.theta < -3 * pi_by_4) ? ZEROTO2PI : PITOPI;
    if(range_select == ZEROTO2PI){
        // update cpos.theta and tpos.theta to new range
        if(tpos.theta < 0) tpos.theta += pi_2;
        if(cpos.theta < 0) cpos.theta += pi_2;
    }

    // start the thread to periodically publish vel_cmd
    to_pub_vel = true;
    std::thread th1{pub_vel_periodic};
    if(abs(radians) > AT){
        float sleep_time {(abs(radians) - AT)/abs(speed)};
        ros::Duration sleep{sleep_time};
        // Start turning the robot
        vel_cmd.angular.z = speed;
        vel_pub.publish(vel_cmd);
        ros::spinOnce();
        sleep.sleep();  // turn radians - at angle withtou any feedback
    }
    // Final Moments
    // slow down for precision
    vel_cmd.angular.z = (speed > AAS) ? AAS : ((speed < -AAS) ? -AAS : speed);
    vel_pub.publish(vel_cmd);
    ros::spinOnce();
    if(radians > 0){ // Turn counter clockwise
        // WHEN cpos.theta first becomes less that tpos.theta and then
        // becomes greater that tpos.theta, that's when the robot should stop
        while(tpos.theta > cpos.theta){
            blink->sleep();
            ros::spinOnce();
        }
    } else {
        while(tpos.theta < cpos.theta){
            blink->sleep();
            ros::spinOnce();
        }
    }
    stop_robot();
    th1.join();
    range_select = PITOPI;  // restore to default range;
    if(log){
        wait_for_update();
        ROS_INFO("[%s] turn command completed!", NODE_NAME);
        std::cout << "Current theta: " << cpos.theta << std::endl;
    }
    return;
}
// --------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    setup(argc, argv);
    ROS_INFO("[%s] USE `rosservice call /turn \"radians: <float>\nspeed: <float>\"` to make the robot turn", NODE_NAME);
    ros::spin();
    wrapup();
    return 0;
}
