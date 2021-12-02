#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <roslearning/FibonacciAction.h>

// CONSTANTS
const char* NODE_NAME {"fibonacci_client"};
const char* ACTION_TOPIC {"fibonacci"};

int main(int argc, char** argv){
    ros::init(argc, argv, NODE_NAME);
    
    // create action client
    // true causes client to spin its own thread
    actionlib::SimpleActionClient<roslearning::FibonacciAction> ac{ACTION_TOPIC, true};

    ROS_INFO("%s: Waiting for action server to start!", NODE_NAME);
    ac.waitForServer(); // will wait for infinite time
    ROS_INFO("%s: Action Server Started, Sending Goal.", NODE_NAME);
    roslearning::FibonacciGoal goal;
    // NOTE FibonacciGoal and not FibonacciActionGoal
    goal.order = 20;
    ac.sendGoal(goal);
    bool finished_before_timeout {ac.waitForResult(ros::Duration(goal.order * 1.1))};
    if(finished_before_timeout){
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("%s: Action Finished, %s", NODE_NAME, state.toString().c_str());
    }
    else{
        ROS_INFO("%s: Action did not finish before the timeout!", NODE_NAME);
    }
    return 0;
}