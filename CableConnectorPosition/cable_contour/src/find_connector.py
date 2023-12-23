#!/usr/bin/env python

"""
ON THE RASPI: roslaunch cable_contour camera.launch 

   0------------------> x (cols) Image Frame
   |
   |        c    Camera frame
   |         o---> x
   |         |
   |         V y
   |
   V y (rows)


SUBSCRIBES TO:
    /camera/color/image_raw: Source image topic
    
PUBLISHES TO:
    /cables/image : image with detected connector and search window
    /cables/image_mask : masking    
    /connector/position_connector : connector position in adimensional values wrt. camera frame

"""

#--- Allow relative importing
if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
    
import sys
import rospy
import cv2
import time

from std_msgs.msg           import String
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
from include.connector_detector  import *


class ConnectorDetector:

    def __init__(self, thr_min, thr_max, blur=10, blob_params=None, detection_window=None):
    
        self.set_threshold(thr_min, thr_max)
        self.set_blur(blur)
        self.set_blob_params(blob_params)
        self.detection_window = detection_window
        
        self._t0 = time.time()
        
        self.blob_point = Point()

        # ROS communication
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)
        print ("<< Subscribed to topic /camera/color/image_raw")

        print (">> Publishing image to image_cable_connector")
        self.image_pub = rospy.Publisher("/cables/image",Image,queue_size=1)
        self.mask_pub = rospy.Publisher("/cables/image_mask",Image,queue_size=1)
        print (">> Publishing position to topic position_connector")
        self.blob_pub  = rospy.Publisher("/connector/position_connector",Point,queue_size=1)

    def set_threshold(self, thr_min, thr_max):
        self._threshold = [thr_min, thr_max]
        
    def set_blur(self, blur):
        self._blur = blur
      
    def set_blob_params(self, blob_params):
        self._blob_params = blob_params
      
    def callback(self,data):
        #--- Assuming image is 1920x1080
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape

        if cols > 60 and rows > 60 :
       
            
            cv_image    = draw_frame(cv_image) # Draw the cordinates frame   
            cv_image    = draw_window(cv_image, self.detection_window, line=3) # Draw the windown analizy
            cv_image    = blur_outside(cv_image, 50, self.detection_window) # Blur the outside of the windows analazing

            #--- Detect blobs from the image
            keypoints, mask, contours   = blob_detect(cv_image, self._threshold[0], self._threshold[1], self._blur,blob_params=self._blob_params, search_window=self.detection_window )

            cv_image    = cv2.drawContours(cv_image, contours, -1, (0, 125, 255), 2) # Draw the contours of the cable        
            cv_image    = draw_keypoints(cv_image, keypoints) # Draw the circles of the points wich is analizyng
             
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, "8UC1"))
            except CvBridgeError as e:
                print(e)            

            for i, keyPoint in enumerate(keypoints):
                #--- Here you can implement some tracking algorithm to filter multiple detections
                #--- We are simply getting the first result
                x = keyPoint.pt[0]
                y = keyPoint.pt[1]
                s = keyPoint.size
                print ("kp %d: s = %3d   x = %3d  y= %3d"%(i, s, x, y))
                
                #--- Find x and y position in camera adimensional frame of the connector
                x, y = get_blob_relative_position(cv_image, keyPoint) 
                
                self.blob_point.x = x
                self.blob_point.y = y
                self.blob_pub.publish(self.blob_point) 
                break
                    
            fps = 1.0/(time.time()-self._t0)
            self._t0 = time.time()
            
def main(args):

    rospy.init_node('connector_detector', anonymous=True)

    # cable
    hsv_min = (0,0,165)
    hsv_max = (255,37, 255) 

    # connector 
    hsv_min = (0,0,180)
    hsv_max = (255,50, 255)
    
    blur     = 1
    min_size = 10
    max_size = 40
    
    #--- detection window respect to camera frame in [x_min, y_min, x_max, y_max] adimensional (0 to 1)
    x_min   = 0.15
    x_max   = 0.95
    y_min   = 0
    y_max   = 1
    
    detection_window = [x_min, y_min, x_max, y_max]
    
    params = cv2.SimpleBlobDetector_Params()
         
    # Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 100;
     
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 100
    params.maxArea = 10000
     
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.1
     
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.87
     
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.1   

    ic = ConnectorDetector(hsv_min, hsv_max, blur, params, detection_window)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
