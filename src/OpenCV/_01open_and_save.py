#! /usr/bin/env python
# the above line is important if you're planning on to run this script on its own without
# as an executable, without passing it as an argument to python interpreter

# You can use rosrun command to run this script though it doesn't use any ros functionalities whatsover

import numpy
# the datastructure that will store the image

import cv2
# opencv module

def main():
    image_name = "Test_1"
    extension = ".png"
    image_path = "/home/coderp/Pictures/"

    print(f"Image Path: {image_path}{image_name}{extension}")
    print("Reading an image from file...")
    img = cv2.imread(image_path + image_name + extension)

    print("Creating a window holder for the image...")
    window_name = image_name
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    print("Displaying the image...")
    cv2.imshow(window_name, img)

    print("Press a key inside the image to make a copy")
    cv2.waitKey(0)

    print("Image copied to the folder ~/Pictures/copy/")
    copy_path = f"{image_path}copy/{image_name}{extension}"
    print(f"Copy Path: {copy_path}")
    # saving the image back to storage
    cv2.imwrite(copy_path, img)


if __name__ == "__main__":
    main()