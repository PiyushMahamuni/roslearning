#! /usr/bin/env python

import numpy as np
import cv2


# It's a incredibly good idea to follow the if main def main idiom

def main():
	image_name = "black_white"
	image_path = "/home/coderp/Pictures/"
	extn = ".jpg"
	print("Reading from an image file")
	img = cv2.imread(image_path+image_name+extn)
	
	print("Display the contents of the image...")
	print(img)
	# with cv2 client library of python, an image is stored in a numpy array.
	# Numpy is a library used for scientific computations of multidimensional arrays
	
	# We can determine several features of the images using numpy array properties
	print(f"Type of the image: {type(img)}")
	# since the img object is an object of class numpy.ndarray, that is what you'll see printed on the terminal
	print(f"Size of the image: {img.size}")
	# img.size returns the total number of individual elements ( aka pixels * channels)
	# length = img.shape[0]
	# width = img.shape[1]
	# channels = img.shape[2]
	# img.size = length * width * channels
	# you can get the lenght of the image with length = len(img) too
	
	# My infrerences - 
	# by default the images are considered to be black and white and each individual pixel is represented as a 
	# list of three values [r_value, g_value, b_value]
	# an image is consists of columns stacking next to each other, so columnts are lists containing these pixels
	# and the final list which contains all these rows together is representing the whole image.
	
	# you can see a single values of single channel from the image with slicing -
	print(img[:,:, 0])
	# there are 3 args in subsript operator,
	# first telling to traverse through all columns of image, second telling to go through each row of each column
	# last one telling to just select the first channel or the 0th index for all pixel values (lists)


if __name__ == "__main__":
	main()
