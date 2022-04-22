import cv2
import numpy as np
import pdb

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
#
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def cd_color_segmentation(img, template):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the top left of the bbox and (x2, y2) is the bottom right of the bbox
	"""
	#bad: 15,17,14,11,5
	#mid:9,6,2
	bounding_box = ((0,0),(0,0))

	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        #colors in hsv
        light_orange = np.array([10, 125, 125])
        dark_orange = np.array([35,255,255])
	
        #light_orange = np.array([5, 200, 200])
        #dark_orange = np.array([35,255,255])
	
	#light_orange = np.array([5, 200, 200])
        #dark_orange = np.array([25,255,255])
        
	#light_orange = np.array([5, 100, 20])
        #dark_orange = np.array([15,255,255])

        mask = cv2.inRange(hsv, light_orange, dark_orange)
        isolated_color = cv2.bitwise_and(img,img, mask= mask)
        #image_print(isolated_color)

        #this is a blank array which is the shape of the image, and all black, which is then going to have a rectangle drawn on it
        blank = np.zeros(img.shape[:2], dtype = "uint8")

        #these are the coordinates which define the rectangle
        topLeft = (0,3*img.shape[0]/8)
        botRight = (img.shape[1],5*img.shape[0]/8)

        #creates a white rectangle the width of the image and 1/4 the height, offset 1/8 up from the bottom, the rest is black
        rectangle_mask = cv2.rectangle(blank,topLeft,botRight,(255,255,255),-1)

        #this calculates the intersection of the color-isolated image with the rectangle, effectively cutting out everything except the
        #sliver of image that we want
        output = cv2.bitwise_and(isolated_color, isolated_color, mask=rectangle_mask)

        # image_print(output)

        #NOW ALL OF THIS SHOULD BE THE EXACT SAME AS THE NORMAL COLOR SEGMENTATION
        gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)

        kernel = np.ones((5,5), np.uint8)
        kernel2 = np.ones((5,5), np.uint8)
        gray = cv2.dilate(gray, kernel2, iterations=1)
        #gray = cv2.erode(gray, kernel, iterations=1)
        
        # create a binary thresholded image
        ret,threshold = cv2.threshold(gray,50,255,cv2.THRESH_BINARY)
        #cv2.adaptiveThreshold(gray,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY,11,2)

        # contours from the thresholded image
        contours = cv2.findContours(threshold, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        shape = gray.copy()
        cv2.drawContours(shape, contours[1], -1, (255,0,0), 2)
        
        x,y,w,h = cv2.boundingRect(contours[0])
        bounding_box = ((x,y),(x+w,y+h))
        #bounding_box = ((x,y+int(5/8)*h)),(x+w,y+int(h*(7/8)))
        cv2.rectangle(img,(x,y),(x+w,y+h),(255,255,255),2)
        cv2.rectangle(gray,(x,y),(x+w,y+h),(255,255,255),2)


        #image_print(img)
        #image_print(gray)
        print(bounding_box)
        # image_print(output)	
        return bounding_box
