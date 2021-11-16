// Tuning of PID gains is still remaining
#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point32.h"
#include <math.h>
#include "roslearning/Goto.h"

// Constants
const char* NODE_NAME {"goto_node"};
const char* VEL_TOPIC {"/turtle1/cmd_vel"};
const char* POSE_TOPIC {"/turtle1/pose"};
const char* GOTO_SERVICE{"/goto"};
const _Float32 LT {0.01};   // linear threshold
const _Float32 pi_by_4{0.785398163};
const _Float32 pi_by_2{pi_by_4 * 2};
const _Float32 pi{pi_by_2 * 2};
const _Float32 pi_2{pi * 2};
const _Float32 AT {(_Float32) (1 * pi_by_4 / 45)};  // angular threshold
const _Float32 xylim1{0.3}, xylim2{11 - xylim1};
#define PITOPI 0
#define ZEROTOPI 1


// Global
ros::Publisher vel_pub;
ros::Subscriber pose_sub;
ros::ServiceServer goto_srvr;
ros::Duration* blink;
#define BLINKDUR 1.0/200.0
turtlesim::Pose cpos;
geometry_msgs::Twist vel_cmd, stop_cmd;
int range_select;
bool to_pub_vel{false};     // controls pub_vel_periodic()
bool pose_updated{false};   // tells if position was updated from last time it was set to false

// CLASSES
class NavParameters{
    public:
    _Float32 kpd;
    _Float32 kpa;
    _Float32 kid;
    _Float32 kia;
    _Float32 kdd;
    _Float32 kda;
    NavParameters(const _Float32& kpd, const _Float32& kpa, const _Float32& kid, const _Float32& kia, const _Float32& kdd, const _Float32& kda):kpd{kpd}, kpa{kpa}, kid{kid}, kia{kia}, kdd{kdd}, kda{kda}{
    }
    NavParameters(){
    }
}np; // global object, stores navigation parameters

// PROTOTYPES
// setups publishers, subscribers, durations, services of this node
inline void setup();
// deletes global objects from heap, wraps up this node
inline void wrapup();
// callback function for POSE_TOPIC
void pose_callback(const turtlesim::Pose::ConstPtr& msg);
// function to stop robot and thread running pub_vel_periodic()
// tells whether given goal location is out of bounds or not
bool is_out_of_bounds(const geometry_msgs::Point32& goal);
void stop_robot();
// waits for update on POSE_TOPIC
void wait_for_update();
// moves the robot towards goal location using PID control
bool goto_goal(const geometry_msgs::Point32& tpos, const NavParameters& np, std::string& reason, bool log = false);
// goto service handler
bool goto_handler(roslearning::Goto::Request& req, roslearning::Goto::Response& res);

// callback function for POSE_TOPIC
void pose_callback(const turtlesim::Pose::ConstPtr& msg){
    cpos.x = msg->x;
    cpos.y = msg->y;
    if(range_select == PITOPI)  cpos.theta = msg->theta;
    else cpos.theta = (msg->theta > 0) ? msg->theta : msg->theta + pi_2;
    pose_updated = true;
}

// // tells whether given goal location is out of bounds or not
bool is_out_of_bounds(const geometry_msgs::Point32& tpos){
    return tpos.x > xylim2 || tpos.x < xylim1 || tpos.y > xylim2 || tpos.y < xylim1;
}

// function to stop robot and thread running pub_vel_periodic()
void stop_robot(){
    to_pub_vel = false;
    vel_cmd = stop_cmd;
    vel_pub.publish(vel_cmd);
    ros::spinOnce();
}

// wait for new update on POSE_TOPIC
void wait_for_update(){
    pose_updated = false;
    try{
        while(!pose_updated){
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

// goes to specified position using proportional controller
bool goto_goal(const geometry_msgs::Point32& tpos, const NavParameters& np, std::string& reason, bool log){
    stop_robot();
    wait_for_update();
    if(is_out_of_bounds(tpos)){
        reason = "Goal location is out of bounds";
        return false;
    }
    if(log){
        ROS_INFO("[%s] Command recieved, goto x= %f, y= %f", NODE_NAME,
        tpos.x, tpos.y);
        std::cout << "Current pos:\n"
                  << "x = " << cpos.x << " y = " << cpos.y << '\n';
    }
    // publish message at 100 Hz per second
    ros::Rate loop_rate{100};
    _Float32 xE, yE, distE, angleE; // proportionality gain distance and angle
    _Float32 ICd{}, ICa{};
    xE = tpos.x - cpos.x;
    yE = tpos.y - cpos.y;
    distE = (_Float32)sqrt(xE * xE + yE * yE);
    angleE = atan2(yE, xE);
    angleE -= cpos.theta;
    angleE = (angleE < -pi) ? angleE + pi_2 : ((angleE > pi) ? angleE - pi_2: angleE);
    do{
        xE = tpos.x - cpos.x;
        yE = tpos.y - cpos.y;
        distE = (_Float32)sqrt(xE * xE + yE * yE);
        angleE = atan2(yE, xE) - cpos.theta;
        angleE = (angleE < -pi) ? angleE + pi_2 : ((angleE > pi) ? angleE - pi_2 : angleE);

        // kick in integral control in last part of approach
        if(distE < 1)
        ICd += (np.kid * distE * cos(angleE)); // integral controller
        // cos - to ensure to add raise integral control only if robot is headed in right direction
        if(angleE < 5 * pi_by_4 / 45)
        ICa += (np.kia * angleE);
        // capping integral controllers to they don't go out of control
        if(ICd > 2) ICd = 2;
        else if(ICd < -2) ICd = -2;
        if(ICa > pi) ICa = pi;
        else if(ICa < -pi) ICa = -pi;

        // Derrivative controller
        // DCd = kdd * (distE - distEp); // rate of change of error, rate is const, 500 Hz
        // DCa = kda * (angleE - angleEp);
        // distEp = distE;
        // angleEp = angleE;

        // PI controller, the robot wouldn's slow down as much in final moments of approach
        // as a stable P controller would
        vel_cmd.linear.x = np.kpd * distE + ICd - np.kdd * vel_cmd.linear.x;
        vel_cmd.angular.z = np.kpa * angleE + ICa - np.kda * vel_cmd.angular.z;
        // angleEp - angleE since we want to oppose rapid change in error as you are approaching
        // your needed anglur orientation
        vel_pub.publish(vel_cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }while((distE > LT || angleE > AT) && ros::ok());

    if(log){
        ROS_INFO("[%s] goto command completed!", NODE_NAME);
        std::cout << "Current location: x = " << cpos.x << " y = " << cpos.y << '\n';
    }
    reason = "Done!";
    return true;
}

// GOTO_SERVICE handler
bool goto_handler(roslearning::Goto::Request& req, roslearning::Goto::Response& res){
    //
    res.success = goto_goal(req.goal, np, res.reason, true);
    return true;
}

// setup the node
inline void setup(int argc, char** argv){
    std::cout << "Setting up " << NODE_NAME << " node\n";
    ros::init(argc, argv, NODE_NAME);
    // static so that NodeHandle won't be deleted until main returns
    static ros::NodeHandle node;
    // setting up publishers
    vel_pub = node.advertise<geometry_msgs::Twist>(VEL_TOPIC, 1);
    // setting up subscribers
    pose_sub = node.subscribe(POSE_TOPIC, 1, pose_callback);
    // setting up services
    goto_srvr = node.advertiseService(GOTO_SERVICE, goto_handler);

    blink = new ros::Duration(BLINKDUR);
    bool success {ros::param::get("~kpd", np.kpd)};
    if(success) success = ros::param::get("~kpa", np.kpa);
    if(success) success = ros::param::get("~kid", np.kid);
    if(success) success = ros::param::get("~kia", np.kia);
    if(success) success = ros::param::get("~kdd", np.kdd);
    if(success) success = ros::param::get("~kda", np.kda);
    if(!success){
        ROS_ERROR("\n[%s] FAILED TO RETRIEVE NAVIGATION PARAMETERS!\n", NODE_NAME);
        ros::shutdown();
        return;
    }
    std::cout << "Node setup complete!\n";
}

// wrap up node
inline void wrapup(){
    delete blink;
}

// main driver
int main(int argc, char** argv){
    setup(argc, argv);
    ROS_INFO("[%s] run `rosservice call /goto \"goal:\n x: <float>\n y: <float>\"` to make the robot move to that location!\n", NODE_NAME);
    ros::spin();
    wrapup();
    return 0;
}