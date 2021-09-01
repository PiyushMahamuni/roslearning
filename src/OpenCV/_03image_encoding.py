import cv2, numpy as np

def main():
    image_name = "Test_1"
    ext = ".png"
    image_dir = "/home/coderp/Pictures/"
    image_path = f"{image_dir}{image_name}{ext}"
    print(f"image_path: {image_path}")
    color_img = cv2.imread(image_path, cv2.IMREAD_COLOR)
    # the cv2.IMREAD_COLOR is flag that tells the cv2.imread() to load the image
    # with standard RGB channels

    print("Displaying the image in the native colour")
    cv2.imshow("Original Image", color_img)
    cv2.moveWindow("Original Image", 0, 0)
    print(f"Color_image.shape: {color_img.shape}")

    height, width, channels = color_img.shape
    print(f"Height: {height}")
    print(f"Width: {width}")
    print(f"Channels: {channels}")

    cv2.waitKey(0)
    cv2.destroyAllWindows()

    print("Splitting the image into three channels")
    blue, green, red = cv2.split(color_img)

    # all of the individual channels are now will be displayed as a grayscale image
    # in of themselves

    print("Displaying the blue channel of the image")
    cv2.imshow("Blue Channel", blue)
    cv2.moveWindow("Blue Channel", 0, 0)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    print("Displaying the green channel of the image")
    cv2.imshow("Green Channel", green)
    cv2.moveWindow("Green Channel", 0, 0)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    print("Displaying the red channel of the image")
    cv2.imshow("Red Channel", red)
    cv2.moveWindow("Red Channel", 0, 0)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    # we loaded the image in a .png format, but we can save it any other format with
    # help of opencv library. Let's save it in HSV format

    print("Splitting the image in Hue Saturation and Value channels...")
    hsv = cv2.cvtColor(color_img, cv2.COLOR_BGR2HSV)
    # cvtColor = convert color image
    # 1st para - image obj (numpy.ndarray)
    # 2nd para - flag indicating conversion to and from

    # splitting into individual channels
    h, s, v = cv2.split(hsv)

    hsv_image = np.concatenate((h, s, v), axis=1)
    # this way you'll see all the hsv channels images stacked left to right
    # first para is a tuple containing the channels, second - I don't know yet what it represents
    cv2.imshow("Hue Saturation Value iamge format", hsv_image)
    cv2.moveWindow("Hue Saturation Value iamge format", 0, 0)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


    # converting an image to grayscale image
    grayscale_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
    cv2.imshow("Grayscale version", grayscale_img)
    cv2.moveWindow("Grayscale version", 0, 0)
    # this conversion is very useful and used oftenly since it is very easy to
    # process a greyscale single channel image than to process 3 channel rgb channels all
    # encoding different features within them
    
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()