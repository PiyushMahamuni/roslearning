#!/usr/bin/env python

import numpy as np, cv2

def filter_color(rgb_image, lower_bound_color, upper_bound_color):
    #convert the image into the HSV color space
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv image", hsv_image)

    #define a mask using the lower and upper bounds of the yellow color 
    mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)

    return mask


def getContours(binary_image):      
    contours, hierarchy = cv2.findContours(binary_image.copy(), 
                                            cv2.RETR_EXTERNAL,
	                                        cv2.CHAIN_APPROX_SIMPLE)
    return contours


def draw_ball_contour(binary_image, rgb_image, contours):
    black_image = np.zeros([binary_image.shape[0], binary_image.shape[1],3],'uint8')
    # could have used single channel grayscale image but then we wouldn't be able to draw the red
    # circles representing the detected tennis balls.

    for c in contours:
        area = cv2.contourArea(c)
        perimeter= cv2.arcLength(c, True)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        x = int(x)
        y = int(y)
        if area > 100:
            cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
            cv2.drawContours(black_image, [c], -1, (150,250,150), 1)
            #paras- image on which to draw the contours, list of countours to draw, fill type, color
            # thickness
            cv2.circle(rgb_image, (x,y), int(radius), (0,0,255), 1)
            cv2.circle(black_image, (x,y), int(radius), (0,0,255), 1)
            cv2.circle(black_image, (x,y), 5, (150,150,255), -1)
            # draws a pink dot at center of contour
            print (f"Area: {area}, Perimeter: {perimeter}")
    print (f"number of contours: {len(contours)}")
    cv2.imshow("RGB Image Contours", rgb_image)
    cv2.imshow("Black Image Contours", black_image)


def main():
    video_path = "/home/phineas/Videos/tennis-ball-video.mp4"
    video_capture = cv2.VideoCapture(video_path) 
    yellowLower =(30, 150, 100)
    yellowUpper = (50, 255, 255)

    waitPeriod = 1000 // 30  # run video at 30 fps
    while True:
        ret, frame = video_capture.read()
        binary_image_mask = filter_color(frame, yellowLower, yellowUpper)
        contours = getContours(binary_image_mask)
        draw_ball_contour(binary_image_mask, frame, contours)
        if cv2.waitKey(waitPeriod) & 0xFF == ord('q'):
            break
    
    video_capture.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
