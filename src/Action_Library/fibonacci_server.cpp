#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "roslearning/FibonacciAction.h"

// CONSTANTS
const char* NODE_NAME {"fibonacci_server"};
const char* ACTION_TOPIC {"fibonacci"};

// GLOBALS

class FibonacciAction{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<roslearning::FibonacciAction> as_;
    // NodeHandle must be created before executing above lines otherwise errors will occur
    std::string action_name_;
    // create messages that are used to publish feedback/result
    roslearning::FibonacciFeedback feedback_;
    roslearning::FibonacciResult result_;

public:
    FibonacciAction(std::string name):
    as_{nh_, name, boost::bind(&FibonacciAction::executeCB, this, _1), false},
    // nodehandle, name of action, binding function and the object to be called on callback, bool auto_start
    action_name_{name}
    {
        as_.start();
        // start the action server
    }

    ~FibonacciAction(void){
        // what is this syntax, passing only void to parameter list of a destructor?
        // I haven't seen this before
    }

    void executeCB(const roslearning::FibonacciGoalConstPtr& goal){
        // helper variables
        ros::Rate r{1};
        bool success {true};

        // push back the seeds for the fibonacci sequence
        feedback_.sequence.clear();
        // clears the sequence array
        feedback_.sequence.push_back(0);
        feedback_.sequence.push_back(1);

        // publish info on the console for the user
        ROS_INFO("%s: Executing, creating fibonacci sequence of order %i, with seeds %i, %i",
        action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);

        // start calculating fibonacci terms
        for(int i{1}; i<=goal->order; i++){
            // check that the preempt has not been requested by the client
            // preempt - taking action in order to prevent an event from happening
            // i.e. check if the client wants to abort requested action
            if(as_.isPreemptRequested() || !ros::ok()){
                ROS_INFO("%s: Preempt Requested!", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                success = false;
                break;
            }
            feedback_.sequence.push_back(feedback_.sequence[i] + feedback_.sequence[i-1]);
            // publish the updated feedback
            as_.publishFeedback(feedback_);

            r.sleep();
            // this sleep is not necessary and is included only for demonstration purpose
        }

        if (success){
            result_.sequence = feedback_.sequence;
            // set the action state to succeeded
            as_.setSucceeded(result_);
            // this also publishes the result
            ROS_INFO("%s: Succeeded", action_name_.c_str());
        }
    }
};


int main(int argc, char** argv){
    ros::init(argc, argv, NODE_NAME);
    FibonacciAction fibonacci {ACTION_TOPIC};
    ros::spin();
    return 0;
}
