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
const _Float32 ALS{0.3}; // Treat as const, don't change value on your own
const _Float32 AAS {static_cast<_Float32>(15 * pi_by_4 / 45)};
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

    // start publishing velocity periodically
    std::thread th1(pub_vel_periodic);
    // allow th1 to keep publishing
    to_pub_vel = true;
    if (abs(distance) > LT)
    {
        // calculate sleep duration
        float temp = ((abs(distance) - LT) / abs(speed));
        ros::Duration sleep{temp};
        // start moving robot
        vel_msg.linear.x = speed;
        vel_pub.publish(vel_msg);
        ros::spinOnce();
        sleep.sleep();
    }

    // Final approach
    vel_msg.linear.x = (vel_msg.linear.x > 0) ? ALS : -ALS;
    // reason why you shouldn't change approach speed even though it is not constant
    vel_pub.publish(vel_msg);
    ros::spinOnce();

    // Decide whether to use x or y cord
    if (abs(cpos.theta) > pi_by_4 && abs(cpos.theta) < 3 * pi_by_4)
    {
        // use y cord
        if (tpos.y > spos.y)
        { // if robot is moving to y value greater than y value at start
            while (tpos.y > cpos.y)
            {
                blink->sleep();
                ros::spinOnce(); // spinOnce at last since you want to
                // make decision out of latest information
            }
        }
        else
        {
            while (tpos.y < cpos.y)
            {
                blink->sleep();
                ros::spinOnce();
            }
        }
    }
    else
    {
        if (tpos.x > spos.x)
        {
            while (tpos.x > cpos.x)
            {
                blink->sleep();
                ros::spinOnce();
            }
        }
        else
        {
            while (tpos.x < cpos.x)
            {
                blink->sleep();
                ros::spinOnce();
            }
        }
    }
    // call to stop_robot automatically make th1 to end its loop and return
    stop_robot();
    // stop velocity publishing through th1s
    th1.join();
    // wait for th1 to join
    if (log)
    {
        ROS_INFO("[%s] Command completed!", NODE_NAME);
        std::cout << "Current position: x = " << cpos.x << " y = " << cpos.y << std::endl;
    }
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
