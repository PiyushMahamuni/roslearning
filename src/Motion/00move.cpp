// NOT WORKING, need to search on threads and timers

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <math.h>
#include <thread>
#include <chrono>

// CONSTANTS
const char *NODE_NAME{"turtle_control"};
const char *VEL_TOPIC{"turtle1/cmd_vel"};
const char *POSE_TOPIC{"turtle1/pose"};
#define VEL_PUB_FREQ 20.0
const _Float32 pi_by_4{0.785398163};
const _Float32 pi_by_2{pi_by_4 * 2};
const _Float32 pi{pi_by_2 * 2};
const _Float32 pi_2{pi * 2};
const _Float32 LT{0.4};         // Linear threshold
const _Float32 AT{5 * static_cast<_Float32>(pi_by_4/45.0)}; // 5 degrees in radians
const _Float32 LLS {1.7};       // limiting linear speed (m/s)
const _Float32 LAS {pi};      // limiting angular speed (rad/s)
_Float32 ALS{0.3}; // Treat as const, don't change value on your own
_Float32 AAS {static_cast<_Float32>(15 * pi_by_4 / 45)};
const _Float32 blink_dur{1.0 / 200};
const _Float32 xylim1{0.3}, xylim2{11 - xylim1};
int range_select {};
#define PITOPI 0
#define ZEROTO2PI 1

// GLOBALS
ros::Publisher vel_pub;
ros::Subscriber pose_sub;
turtlesim::Pose cpos, tpos;
geometry_msgs::Twist vel_msg, stop_msg;
ros::Duration *blink;
bool updated_pose{false};
bool to_pub_vel{false};

// returns true if tpos.theta is out of limits
bool is_crashing()
{
    return tpos.x > xylim2 || tpos.x < xylim1 || tpos.y > xylim2 || tpos.y < xylim1;
}

// function to call periodically to publish vel_msg on vel_topic
void pub_vel_periodic()
{
    auto loop_idle_time = std::chrono::microseconds(static_cast<long long int>(1000000 * 1.0 / VEL_PUB_FREQ));
    auto start = std::chrono::high_resolution_clock::now();
    u_int32_t count{0};
    while (to_pub_vel)
    {
        vel_pub.publish(vel_msg);
        ros::spinOnce();
        count++;
        std::this_thread::sleep_until(start + count * loop_idle_time);
    }
}

// pose_sub callback functin
void pose_sub_callback(const turtlesim::Pose::ConstPtr &msg)
{
    cpos.x = msg->x;
    cpos.y = msg->y;
    if(range_select == PITOPI)
        cpos.theta = msg->theta;
    else
        cpos.theta = (msg->theta < 0) ? (pi_2 + msg->theta) : msg->theta;
    updated_pose = true;
}

// wait for update on pose topic
inline void wait_for_update(){
    updated_pose = false;
    while(!updated_pose){
        blink->sleep();
        ros::spinOnce();
    }
    return;
}

// function to stop robot
inline void stop_robot()
{
    to_pub_vel = false;
    vel_msg = stop_msg;
    vel_pub.publish(vel_msg);
    ros::spinOnce();
}

// setting up this node
inline void setup(int argc, char **argv)
{
    std::cout << "Setting up node..." << std::endl;
    // Initialize node
    ros::init(argc, argv, NODE_NAME);
    // Initialize node handle
    static ros::NodeHandle node;
    // Initialize pose subscriber
    pose_sub = node.subscribe(POSE_TOPIC, 10, pose_sub_callback);
    // Initialize vel publisher
    vel_pub = node.advertise<geometry_msgs::Twist>(VEL_TOPIC, 1);
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
inline void wrapup()
{
    // stop robot
    stop_robot();
    // stop all timers
    // delete any Dynamically Allocated Memory
    delete blink;
}


// MOTION -------------------------------------------------------------------------------------------
// move function
bool move(_Float32 distance, _Float32 speed, bool log = false)
{   
    // make sure that robot is stopped
    stop_robot();
    if((speed == abs(speed)) == 0){
        ROS_ERROR("[%s] Invalid linear speed! Aborting operation", NODE_NAME);
        return false;
    }
    else if(speed > LLS){
        ROS_INFO("[%s] Requested speed is greater than limiting [%f] speed", NODE_NAME, LLS);
        return false;
    }
    // avoid conflicting speed and distance values
    speed = (distance > 0) ? speed : -speed;
    wait_for_update();
    // calculate tpos, store starting pose
    turtlesim::Pose spos;
    tpos.x = (spos.x = cpos.x) + distance * cos(cpos.theta);
    tpos.y = (spos.y = cpos.y) + distance * sin(cpos.theta);

    if (is_crashing())
    {
        ROS_INFO("[%s] Invalid Operation, robot will run into wall!", NODE_NAME);
        return false;
    }

    if (log)
    {
        ROS_INFO("[%s] Command recieved to make the robot move through %f distance with %f speed",
                 NODE_NAME, distance, speed);
        std::cout << "Current position: x = " << cpos.x << " y = " << cpos.y << std::endl
                  << "Target position:  x = " << tpos.x << " y = " << tpos.y << std::endl
                  << "Enter any key to continue, Enter abort to abort: ";
        std::string choice;
        std::getline(std::cin, choice);
        if (choice == "abort")
        {
            ROS_INFO("[%s] User aborted the operation!", NODE_NAME);
            return false;
        }
    }

    // calculate sleep duration
    bool to_sleep{abs(distance) > LT}; // is required later

    ros::Duration *sleep{nullptr};
    std::thread *th1{nullptr};
    if (abs(distance) > LT)
    {
        float temp = ((abs(distance) - LT) / abs(speed));
        if (log)
            std::cout << "Sleeping time: " << temp << std::endl;

        // allow th1 to keep publishing
        to_pub_vel = true;
        // start another thread, publishing velocity periodically
        th1 = new std::thread(pub_vel_periodic);
        sleep = new ros::Duration(temp);
    }
    // start moving robot
    vel_msg.linear.x = speed;
    vel_pub.publish(vel_msg);
    ros::spinOnce();

    if (to_sleep)
    {
        if (log)
            std::cout << "Sleeping..." << std::endl;
        sleep->sleep();
        if (log)
            std::cout << "Awake..." << std::endl;
    }
    // slow down if needed
    if (abs(vel_msg.linear.x) > ALS)
    {
        vel_msg.linear.x = (vel_msg.linear.x > 0) ? ALS : -ALS;
        // reason why you shouldn't change approach speed even though it is not constant
        vel_pub.publish(vel_msg);
        ros::spinOnce();
    }

    // Decide whether to use x or y cord
    if (abs(cpos.theta) > pi_by_4 && abs(cpos.theta) < 3 * pi_by_4)
    {
        // use y cord
        if (tpos.y > spos.y)
        { // if robot is moving to y value greater than y value at start
            while (tpos.y > cpos.y)
            {
                ros::spinOnce();
                blink->sleep();
            }
        }
        else
        {
            while (tpos.y < cpos.y)
            {
                ros::spinOnce();
                blink->sleep();
            }
        }
    }
    else
    {
        if (tpos.x > spos.x)
        {
            while (tpos.x > cpos.x)
            {
                ros::spinOnce();
                blink->sleep();
            }
        }
        else
        {
            while (tpos.x < cpos.x)
            {
                ros::spinOnce();
                blink->sleep();
            }
        }
    }
    stop_robot();
    if (log)
        ROS_INFO("[%s] Stopped Robot, waiting for th1 thread", NODE_NAME);
    // stop velocity publishing through th1s
    th1->join();
    // wait for th1 to join

    delete th1;
    if (log)
    {
        ROS_INFO("[%s] Command completed!", NODE_NAME);
        std::cout << "Current position: x = " << cpos.x << " y = " << cpos.y << std::endl;
    }
    delete sleep;
    return true;
}

inline void forward(const _Float32& dist, const _Float32& speed, bool log = false){
    move(dist, speed, log);
    return;
}

inline void backward(const _Float32& dist, const _Float32& speed, bool log = false){
    move(-dist, speed, false);
}
// --------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cout << "Usage: " << argv[0] << " [distance] [speed]" << std::endl;
        return 1;
    }
    setup(argc, argv);
    move((_Float32)atof(argv[1]), (_Float32)atof(argv[2]), true);
    wrapup();
    return 0;
}
