#include "opencv2/opencv.hpp"

// You need to add these lines for a cpp node needing opencv library in ROS-
// find_package(OpenCV)
// add_executable(xyz path_of_node)
// target_link_libraries(xyz ${catkin_LIBRARIES})   # AS USUAL
// target_link_libraries(xyz ${OpenCV_LIBRARIES})   # Need this line too

// CONSTANTS
const uint FRAME_FREQ {30};

int main(int argc, char** argv){
    if(argc != 2){
        std::cout << "Usage: " << argv[0] << " [camera device index]\n";
        return -1;
    }
    cv::VideoCapture webcam(atoi(argv[1]));
    // The first attached camera/video device Usually the webcam

    if(!webcam.isOpened())
        return -1;
    
    uint wait = {1000 / FRAME_FREQ};
    cv::Mat edges;
    cv::namedWindow("edges", 1);
    while(true){
        cv::Mat frame;
        webcam >> frame;

    
        // read a frame from the WebCam or the first video device
        cv::cvtColor(frame, edges, cv::COLOR_BGR2GRAY);
        // source, destination, conversion

        cv::imshow("edges", edges);
        if(cv::waitKey(wait) >= 0) break;
    }
    cv::destroyAllWindows();
    webcam.release();
    return 0;
}