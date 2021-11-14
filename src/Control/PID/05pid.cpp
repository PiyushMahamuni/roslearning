#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"

// CONSTANTS
const char* NODE_NAME{"/pidLinController"};
const char* VEL_TOPIC{"/turtle1/cmd_vel"};
const char* POSE_TOPIC{"/turtle1/pose"};
const _Float32 LT {0.01}; // linear threshold
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
ros::NodeHandle* node{nullptr};

// setup this node
inline void setup(int argc, char** argv);
// wrapup this node
inline void wrapup();
// callback function for pose_sub
void pose_callback(const turtlesim::Pose::ConstPtr& msg);
// function implementing proportional-integral-derivative linear controller
void pidLineController(_Float32 x, _Float32 kpd, _Float32 kid, _Float32 kdd, _Float32 m, bool log = false);
// stops the turtlebot
void stop_robot();
// wait for pose updated
void waitPoseUpdate();

int main(int argc, char** argv){
    setup(argc, argv);
    std::cout << "Info -------------------------" << std::endl
    << "This program works along with turtlesim_node, emulating that robot has\n"
    << "Given mass `m` it implements a porportional-integral-derrivative cotroller\n"
    << "whose output is a `force` Which is imparted on body of robot to make it move\n"
    << "Note, this program doesn't take in account the angle of robot and assumes it is at 0 rad\n";
    
    float x0, m, kpd, kid, kdd;
    bool success{true};

    success = success && node->getParam("x0", x0);
    if(success)
        success = success && node->getParam("m", m);
    if(success)
        success = success && node->getParam("kpd", kpd);
    if(success)
        success = success && node->getParam("kid", kid);
    if(success)
        success = success && node->getParam("kdd", kdd);
    if(!success){
        ROS_INFO("[%s] Couldn't Retrieve all parameters!", NODE_NAME);
        ros::shutdown();
        return 1;
    }
    pidLineController(x0, kpd, kid, kdd, m, true);
    wrapup();
    return 0;
}

// setup this node
inline void setup(int argc, char** argv){
    // setting up globals
    vel_msg.linear.x = vel_msg.linear.y = vel_msg.linear.z =
    vel_msg.angular.x = vel_msg.angular.y = vel_msg.angular.z = 0;
    stop_msg = vel_msg;
    ros::init(argc, argv, NODE_NAME);
    node = new ros::NodeHandle("~");
    // initialize POSE_TOPIC subscriber
    pose_sub = node->subscribe(POSE_TOPIC, 1, pose_callback);
    // initialize VEL_TOPIC publisher
    vel_pub = node->advertise<geometry_msgs::Twist>(VEL_TOPIC, 1);
    // initialize blink
    blink = new ros::Rate(blinkFreq);
    return;
}

// wrapup this node
inline void wrapup(){
    delete blink;
    delete node;
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

// function implementing proportional-integral-derivative linear controller
// log is default parameter having false value
void pidLineController(_Float32 x0, _Float32 kpd, _Float32 kid, _Float32 kdd, _Float32 m, bool log){
    // this controls the force acted on robot, not velocity of robot
    stop_robot();
    waitPoseUpdate();
    if(log){
        ROS_INFO("[%s] Command recieved for Linear PID control", NODE_NAME);
        std::cout << "m = " << m << " x0 = " << x0 << " kpd = " << kpd << " kid = " << kid
                  << " kdd = " << kdd << '\n'
                  << "Enter any key to continue, abort to abort: ";
        std::string choice;
        std::getline(std::cin, choice);
        if(choice == "abort"){
            ROS_INFO("[%s] Command aborted by user!", NODE_NAME);
            return;
        }
        std::cout << "Enter ctrl+c to stop the controller!\n";
    }
    ros::Rate ControllerRate {controllerFreq};
    _Float32 loop_dur {(_Float32)ControllerRate.expectedCycleTime().toSec()};
    x0 = abs(x0); // there aren't any -ve x values on screen of turtlesim simulator
    kdd /= loop_dur;
    _Float32 dx{x0 - cpos.x}, pv{0}, cv{}, fi{}, K{loop_dur/(2*m)},
    force{}; // prev velocity, current velocity

    // playground
    /*
        Let s be the relative displacement of robot w.r.t. target/desired location/point
        let x0 be the desired value, x be the current position
        relative displacement s = x - x0 = -error, error = x0 - x
        Proportional controller -
        it acts like a spring
        fp = kpd * error = kpd * _s2;
        Integral controller -
        dfi = -kid * s;
        fi += dfi;
        ---------> fi += kid * _s, in loop we use _s2 for current displacement value(negated)
        ---------> fi += kid * _s2;
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
        f = fp + fd + fi
    */
    // ~playground
    do{
        fi += kid * dx;
        force = kpd * dx - kdd * vel_msg.linear.x + fi;
        cv = force * K;
        vel_msg.linear.x += (pv + cv);
        pv = cv;
        vel_pub.publish(vel_msg);
        blink->sleep();
        ros::spinOnce();
        dx = x0 - cpos.x;
    }while(ros::ok());
    if(log) ROS_INFO("[%s] PID Linear Controller stopped!", NODE_NAME);
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
        try{    
            blink->sleep();
            ros::spinOnce();
        }
        catch(ros::Exception& e){
            ROS_INFO("[%s] Exception: %s", NODE_NAME, e.what());
            ros::shutdown();
        }
    }
}