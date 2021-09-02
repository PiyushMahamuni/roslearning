import numpy as np, cv2

def main():
    video_capture = cv2.VideoCapture(0)
    # the pos parameter tells to connect to first video device connected to the computer,
    # you can change that number to connect to respective video device
    # Optionally you can also pass the path of a video file to the VideoCapture object

    while True:
        ret, frame = video_capture.read()
        # reading a frame at a time, since it is a video

        frame = frame[:, -1::-1, :]
        # mirrroring the image
        # you can do any type of image processing you want on each individual imange
        # before displaying it

        frame = cv2.resize(frame, (0,0), fx=1.3, fy=1.3)
        # ressize it to increase width and height by 30 percent

        cv2.imshow("Frame", frame)
        
        # cv2.waitKey waits for given milliseconds and if any key is pressed, it will
        # return true, otherwise false
        # if you use `and` instead of `&` below, it doesn't work
        if cv2.waitKey(1000//60) & 0xFF == ord('q'):
            break
    
    video_capture.release()
    # release the media file or the video device

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()