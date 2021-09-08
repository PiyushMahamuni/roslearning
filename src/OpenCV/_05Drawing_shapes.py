import numpy as np, cv2


def main():
	image = np.zeros((512, 512, 3), np.uint8)

	# np.zeroes returns a n-dimensional array of given shape, here it is an image of 512*512 with 3 channels
	# so it's a 3 dimensional array
	# the last parameter tells the type of `individual` element from the array. here it is unsigned 8 bin integer
	# the one that can store values from 256 distinct values ( 0 to 255 )
	# since all the channels of all the pixels have 0 value, the image is pitch dark
	
	# drawing a line on this complete dark image -
	cv2.line(image, (0,0), (511, 511), (255, 255, 255), 5)
    # origin is at left top corner, 511,511 is right bottom corner
	# first para - the image on which you want to draw the line
	# second para - tuple indicating starting point for the line
	# third para - tuple indicating end point for the line
	# 4th para - RGB values for pixels of line
	# 5th para - thickness
	
	cv2.rectangle(image, (384, 0), (510, 128), (0, 255, 0), 3)
    # 2nd, 3rd para - cordinate of opposite vertices of rectangle
    # 4th para - pixel color RGB values
    # 5th para - thickness
	
	cv2.ellipse(image, (256, 256), (100, 50), 0, 0, 360, (255, 0, 255), -1)
    # elipse, 2nd para - geometric center, 3rd pair of minor and major radii
    # 5th para -  start angle ( angles are measured clockwise 0 pointing to right)
    # e.g. 45 points to bottom right
    # 6th para - end angle
    # 7th para - BGR values, you can pass single value too, but then it will only print
    # shades of blue
    # 8th para - -1 to fill, 0 for only border

    
	pts = np.array([[10,5],[20,30],[70,20],[50,10]], np.int32)
	pts = pts.reshape((-1, 1, 2))
	cv2.polylines(image, [pts], True, (0, 255, 255))
	
	font = cv2.FONT_HERSHEY_SIMPLEX
	cv2.putText(image, 'ROS, OpenCV', (10, 500), font, 2, (255, 255, 255), 2, cv2.LINE_AA)
	
	cv2.imshow("Image Panel", image)
	
	cv2.waitKey(0)
	cv2.destroyAllWindows()


if __name__ == "__main__":
	main()