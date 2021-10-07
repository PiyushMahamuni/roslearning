#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// CONSTANTS
const char *NODE_NAME{"frame_a_to_frame_b_broadcaster"};
const char *PARENT_FRAME{"frame_a"};
const char *CHILD_FRAME{"frame_b"};

// GLOBALS
tf::TransformBroadcaster *transform_broadcaster;
ros::NodeHandle *nh;
tf::Transform transform;

// setup this node
void setup(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    nh = new ros::NodeHandle();
    transform_broadcaster = new tf::TransformBroadcaster();
}

void wrap_up()
{
    delete nh;
    delete transform_broadcaster;
}

int main(int argc, char **argv)
{
    setup(argc, argv);
    ros::Rate loop_rate(10);
    while (nh->ok())
    {
        transform.setOrigin(tf::Vector3(1.0, 2.0, 3.0));
        transform.setRotation(tf::createQuaternionFromRPY(0.3, 0.2, 0.1));
        transform_broadcaster->sendTransform(tf::StampedTransform(transform, ros::Time::now(), PARENT_FRAME, CHILD_FRAME));
        loop_rate.sleep();
    }
    wrap_up();
    return 0;
}

// NOW RUN THE FOLLOWING COMMANDS AFTER BUILDING THIS PROGRAM -
// rosrun roslearing _05Transformation_Broadcaster
// rosrun tf2_tools echo frame_a frame_b
// rosrun tf2_tools view_frames.py
// evince frames.pdf