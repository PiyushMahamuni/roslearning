#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <image_transport/image_transport.h>

// CONSTANTS
const char* NODE_NAME {"bridge"};
const char* IMAGE_IN_TOPIC {"/image"};
const char* IMAGE_OUT_TOPIC {"bridge/converted_image"};

// TO be able to compile this node or any node that uses cv_bridge and image_transport library
// You need to add these in ## Find catkin macros and Libraries section in CMakeList.txt
// Then link the executable against both catkin libraries and opencv libraries


// Launch this node wth _01streaming_video.py from this package

// The image format used to publish an iamge in ROS and the format the opencv client library
// requires are different.

// The library CvBridge takes care of this conversion from images recieved on topics essentially as a
// big array of integers to respective Matrixes the opencv needs to process images

// ALWAYS UPDATE YOUR OPENCV AND CVBRIDGE LIBRARIES


// GLOBALS
ros::Publisher img_pub;


// Defining a ImageConverter class
class ImageConverter{
    private:
    ros::NodeHandle& nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    std::string winname;

    public:
    ImageConverter(ros::NodeHandle& nh, std::string winname) : nh_{nh}, winname{winname}, it_{nh_}{
        // subscirbe to input video feed and publish output video feed
        image_sub_ = it_.subscribe(IMAGE_IN_TOPIC, 1, &ImageConverter::imageCallback, this);
        image_pub_ = it_.advertise(IMAGE_OUT_TOPIC, 1);

        cv::namedWindow(winname);
    }

    ~ImageConverter(){
        cv::destroyWindow(winname);

    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& image){
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("CV Bridge Exception : %s", e.what());
            return;
        }
        // Do some processing on image
        // Drawing a circle
        if(cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60){
            cv::circle(cv_ptr->image, cv::Point(cv_ptr->image.cols/2, cv_ptr->image.rows/2), 10, CV_RGB(255, 0, 0), -1);
        }

        cv::Mat grayscale;
        cv::Mat thresh_image;
        cv::Mat hsv_image;
        cv::cvtColor(cv_ptr->image, grayscale, cv::COLOR_BGR2GRAY);
        cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

        cv::adaptiveThreshold(grayscale, thresh_image, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 7, 0);

        // Update GUI window
        cv::imshow(winname, cv_ptr->image);
        cv::imshow(winname+" in graysclae", grayscale);
        cv::imshow(winname+" thresholded", thresh_image);
        cv::imshow(winname+" hsv", hsv_image);

        
        cv::waitKey(3);
        // output modified video stream

        // CvImagePtr::toImageMsg converts opencv image to ROS image msg
        image_pub_.publish(cv_ptr->toImageMsg());

    }
};

int main(int argc, char** argv){
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle node;
    ImageConverter ic{node, "Image Converter"};
    ros::spin();
    return 0;
}