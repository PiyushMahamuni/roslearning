// Aim: To lean how to open and save images in cpp with opencv library
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// CONSTANTS
std::string image_dir {"/home/coderp/Pictures/"};
std::string image_name  {"Test_1.png"};
// It's good idea to absolutely specify the image path to be able to run this node
// that is executable from any folder

int main(int argc, char **argv){
    // Load image
    cv::Mat image{cv::imread((image_dir+image_name).c_str(), cv::IMREAD_COLOR)};

    // check for valid image file or if it is loaded in memory
    if(!image.data){
        std::cout << "Could not open or find the image\n";
        return -1;
    }


    // Display Image
    cv::namedWindow(image_name.c_str(), cv::WINDOW_AUTOSIZE);
    cv::imshow(image_name.c_str(), image);

    // Save i.e. Write image
    cv::imwrite((image_dir + "/copy/Test_1_cpp.jpg").c_str(), image);

    cv::waitKey(0);
    cv::destroyAllWindows();
    
    return 0;
}