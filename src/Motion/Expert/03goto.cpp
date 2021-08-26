// Tuning of PID gains is still remaining
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
bool goto_goal(turtlesim::Pose tpos, _Float32 kpd, _Float32 kpa, _Float32 kid,
              _Float32 kia, _Float32 kdd, _Float32 kda, bool log = false){
    stop_robot();
    wait_for_update();
    if(log){
        ROS_INFO("[%s] Command recieved, goto %f %f %f", NODE_NAME,
        tpos.x, tpos.y, tpos.theta);
        std::cout << "Current pos:\n"
                  << "x = " << cpos.x << " y = " << cpos.y << '\n'
                  << "Enter any key to continue, abort to abort: ";
        std::string choice;
        std::getline(std::cin, choice);
        if(choice == "abort"){
            ROS_INFO("[%s] Command aborted by user!", NODE_NAME);
            return false;
        }
    }
    // publish message at 50 Hz per second
    ros::Rate loop_rate{100};
    _Float32 xE, yE, distE, angleE; // proportionality gain distance and angle
    _Float32 ICd{}, ICa{};
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
        angleE -= cpos.theta;
        if(angleE < -pi) angleE += pi_2;
        else if(angleE > pi) angleE -= pi_2;
    _Float32 distEp{distE}, angleEp{angleE}; // required for Derrivative control
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
        angleE -= cpos.theta;
        if(angleE < -pi) angleE += pi_2;
        else if(angleE > pi) angleE -= pi_2;

        // kick in integral control in last part of approach
        if(distE < 1)
        ICd += (kid * distE * cos(angleE)); // integral controller
        if(angleE < 5 * pi_by_4 / 45)
        ICa += (kia * angleE);
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
        vel_msg.linear.x = kpd * distE + ICd + kdd * (distE - distEp);
        vel_msg.angular.z = kpa * angleE + ICa + kda * (angleEp - angleE);
        // angleEp - angleE since we want to oppose rapid change in error as you are approaching
        // your needed anglur orientation
        distEp = distE;
        angleEp = angleE;
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
    if(argc != 9){
        std::cout << "Usage: " << argv[0] << " x, y, kpd, kpa, kid, kia, kdd, kda OR\n"
                  << "Type recommeded to use recommended gain values\n";
        return 1;
    }
    setup(argc, argv);
    turtlesim::Pose target;
    target.x = (_Float32)atof(argv[1]);
    target.y = (_Float32)atof(argv[2]);
    _Float32 kpd, kpa, kid, kia, kdd, kda;
    if(std::string{"recommended"} == std::string{argv[3]}){
        // assign default values to all gains
    }
    else{
        kpd = (_Float32)atof(argv[3]);
        kpa = (_Float32)atof(argv[4]);
        kid = (_Float32)atof(argv[4]);
        kia = (_Float32)atof(argv[4]);
        kdd = (_Float32)atof(argv[4]);
        kda = (_Float32)atof(argv[4]);
    }
    goto_goal(target, kpd, kpa, kid, kia, kdd, kda, true);
    wrapup();
    return 0;
}