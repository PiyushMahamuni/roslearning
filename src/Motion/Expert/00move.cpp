// NOT WORKING, need to search on threads and timers

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"
#include <math.h>
#include <thread>
#include <chrono>
#include "roslearning/Move.h"

// CONSTANTS
const char *NODE_NAME{"turtle_control_move"};
const char *VEL_TOPIC{"/turtle1/cmd_vel"};
const char *POSE_TOPIC{"/turtle1/pose"};
const char *MOVE_TOPIC{"/move"};

#define VEL_PUB_FREQ 20.0
const _Float32 pi_by_4{0.785398163};
const _Float32 LT{0.5};     // Linear threshold
const _Float32 LLS {1.7};   // limiting linear speed (m/s)
const _Float32 ALS {0.2};   // Treat as const, don't change value on your own
// approaching angular and linear speeds
const _Float32 blink_dur{1.0 / 200};
const _Float32 xylim1{0.3}, xylim2{11 - xylim1};


// GLOBALS
ros::Publisher vel_pub;
ros::Subscriber pose_sub;
ros::ServiceServer move_srvr;
turtlesim::Pose cpos, tpos;
geometry_msgs::Twist vel_cmd, stop_cmd;
ros::Duration *blink;
bool updated_pose{false};
bool to_pub_vel{false};

// PROTOTYPES
// setup this node
inline void setup();
// wrapup this node
inline void wrapup();
// checks if the set target is within acceptable limits
bool is_crashin();
// publishes velocity command periodically
void pub_vel_periodic();
// pose_sub callback function
void pose_sub_callback(const turtlesim::Pose::ConstPtr& msg);
// waits for the position to be updated from the last time updated_pose global was set to false
inline void wait_for_update();
// stops the robot
inline void stop_robot();
// move service handler
bool handle_move(roslearning::Move::Request& req, roslearning::Move::Response& res);
// moves the robot through given distance with given speed
bool move(float distance, float speed, std::string& reason, bool log = false);
// moves the robot forward through given distance with given speed
inline void forward(const _Float32& dist, const _Float32& speed, bool log = false);
// moves the robot backward through given distance with given speed
inline void backward(const _Float32& dist, const _Float32& speed, bool log = false);


// returns true if tpos.theta is out of limits
bool is_crashing()
{
    return tpos.x > xylim2 || tpos.x < xylim1 || tpos.y > xylim2 || tpos.y < xylim1;
}

// function to call periodically to publish vel_cmd on vel_topic
void pub_vel_periodic()
{
    auto loop_idle_time = std::chrono::microseconds(static_cast<long long int>(1000000 * 1.0 / VEL_PUB_FREQ));
    auto start = std::chrono::high_resolution_clock::now();
    u_int32_t count{0};
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
    cpos.theta = msg->theta;
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

// move service handler
bool handle_move(roslearning::Move::Request& req, roslearning::Move::Response& res){
    wait_for_update();
    res.success = move(req.distance, req.speed, res.reason, true);
    return true;
    // the res.success response will tell if the operation was unsuccessful, don't need to
    // respong with an error: b'' with rosservice APIs, hence always returning true
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
    stop_cmd.linear.x = 0;
    stop_cmd.linear.y = 0;
    stop_cmd.linear.z = 0;
    stop_cmd.angular.x = 0;
    stop_cmd.angular.y = 0;
    stop_cmd.angular.z = 0;
    // setup blink duration

    // setup move service
    move_srvr = node.advertiseService(MOVE_TOPIC, handle_move);
    blink = new ros::Duration(blink_dur);
    std::cout << "Setting up node complete..." << std::endl;
    ROS_INFO("\n[%s] Use `rosservice call /move \"distance: <float> speed: <float>\"` to make the robot move!\n", NODE_NAME);
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
    return;
}


// MOTION -------------------------------------------------------------------------------------------
// move function
bool move(_Float32 distance, _Float32 speed, std::string& reason, bool log)
{   
    // make sure that robot is stopped
    reason = "Done!";
    stop_robot();
    if((speed = abs(speed)) == 0){
        reason = "Invalid Linear Speed";
        ROS_ERROR("[%s] Invalid linear speed! Aborting operation", NODE_NAME);
        return false;
    }
    else if(speed > LLS){
        reason = "speed > Limiting Speed";
        ROS_INFO("[%s] Requested speed is greater than limiting [%f] speed", NODE_NAME, LLS);
        return false;
    }
    else if(distance == 0){
        reason = "distance is 0.0";
        ROS_INFO("[%s] distance to move through is 0!", NODE_NAME);
        return true;
    }
    wait_for_update();
    // calculate tpos, store starting pose
    turtlesim::Pose spos;
    tpos.x = (spos.x = cpos.x) + distance * cos(cpos.theta);
    tpos.y = (spos.y = cpos.y) + distance * sin(cpos.theta);

    if (is_crashing())
    {
        reason = "Robot will run into wall";
        ROS_INFO("[%s] Invalid Operation, robot will run into wall!", NODE_NAME);
        return false;
    }

    if (log)
    {
        ROS_INFO("[%s] Command recieved to make the robot move through %f distance with %f speed",
                 NODE_NAME, distance, speed);
        std::cout << "Current position: x = " << cpos.x << " y = " << cpos.y << std::endl
                  << "Target position:  x = " << tpos.x << " y = " << tpos.y << std::endl;
    }
    // avoid conflicting speed and distance values
    speed = (distance > 0) ? speed : -speed;

    // allow th1 to keep publishing
    to_pub_vel = true;
    bool use_y {abs(cpos.theta) > pi_by_4 && abs(cpos.theta) < 3 * pi_by_4};
    // start publishing velocity periodically
    std::thread th1(pub_vel_periodic);
    if (abs(distance) > LT)
    {
        // calculate sleep duration
        float temp = ((abs(distance) - LT) / abs(speed));
        ros::Duration sleep{temp};
        // start moving robot
        vel_cmd.linear.x = speed;
        vel_pub.publish(vel_cmd);
        ros::spinOnce();
        sleep.sleep();
    }
    // Final approach
    vel_cmd.linear.x = (speed > ALS) ? ALS : ((speed < -ALS) ? -ALS : speed);
    // ALS - APPROACHING LINEAR SPEED
    
    vel_pub.publish(vel_cmd);
    ros::spinOnce();

    // Decide whether to use x or y cord
    if (use_y)
    {
        // use y cord
        if (tpos.y > spos.y)
        { // if robot is moving to y value greater than y value at start
            while (tpos.y > cpos.y)
            {
                blink->sleep();
                ros::spinOnce();
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
        ROS_INFO("[%s] move Command completed!", NODE_NAME);
        std::cout << "Current position: x = " << cpos.x << " y = " << cpos.y << std::endl;
    }
    return true;
}
// --------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    setup(argc, argv);
    ros::Rate loop_rate(30);
    ros::spin();
    wrapup();
    return 0;
}
