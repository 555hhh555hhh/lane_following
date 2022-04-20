#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy, math, time
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:

        def __init__(self):

                self.bridge = cv_bridge.CvBridge()

                self.image_sub = rospy.Subscriber('camera/image',
                        Image, self.image_callback)

                self.cmd_vel_pub = rospy.Publisher('cmd_vel',
                        Twist, queue_size=10)

                self.twist = Twist()
		self.stop_flag = False

        def image_callback(self, msg):

                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

                lower_yellow = numpy.array([ 10, 40, 40])
                upper_yellow = numpy.array([40, 255, 250])

                lower_white = numpy.array([0, 0, 221])
#lower_white = numpy.array([0, 0, 80])
                upper_white = numpy.array([180, 43, 255])
                
                mask1 = cv2.inRange(hsv, lower_yellow, upper_yellow)
                mask2 = cv2.inRange(hsv, lower_white, upper_white)

                h, w, d = image.shape

		ori = numpy.array([[50, 210], [250,210], [110, 160],[200, 160]])
                bev = numpy.array([[80, 240], [180,240], [80, 60],[180, 60]])
                
                H, status = cv2.findHomography(ori, bev)
                bev_img = cv2.warpPerspective(image, H, (w,h))

                search_top = 2*h/3
#search_top = 2*h/3
                mask1[0:search_top, 0:w] = 0
                mask2[0:search_top, 0:w] = 0

                M1 = cv2.moments(mask1)
                M2 = cv2.moments(mask2)

                if M1['m00'] > 0:
                    cx1 = int(M1['m10']/M1['m00'])
                    cy1 = int(M1['m01']/M1['m00'])

                    cx2 = int(M2['m10']/M2['m00'])
                    cy2 = int(M2['m01']/M2['m00'])

                    fpt_x = (cx1 + cx2)/2
                    fpt_y = (cy1 + cy2)/2 + 2*h/3

                    cv2.circle(image, (cx1, cy1), 10, (0,255,255), -1)
                    cv2.circle(image, (cx2, cy2), 10, (255,255,255), -1)
                    cv2.circle(image, (fpt_x, fpt_y), 30, (0,0,255), -1)
#cv2.circle(image, (fpt_x, fpt_y), 10, (128,128,128), -1)

                    err = w/2 - fpt_x
		    #print err

                    self.twist.linear.x = 0.4
                    #self.twist.angular.z = (err*90.0/160)/15
                    #self.twist.linear.x = 0.45-abs(err)/75
                    self.twist.angular.z = (err*90.0/160)/10
		    self.cmd_vel_pub.publish(self.twist)
#aruco
		aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
		param = aruco.DetectorParameters_create()
                corners, markerID, rejected = aruco.detectMarkers(image, aruco_dict, parameters=param)

                matrix = numpy.array([[322.0704122808738, 0, 199.2680620421962],[ 0, 320.8673986158544, 155.2533082600705],[ 0, 0, 1]])
                dist = numpy.array([[0.1639958233797625, -0.271840030972792, 0.001055841660100477, -0.00166555973740089, 0]]) 

		if len(corners)>0 and not self.stop_flag:
		    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.0125, matrix, dist)
                    (rvec-tvec).any()
		    for i in range(rvec.shape[0]):
			aruco.drawDetectedMarkers(image, corners,markerID)
		    distance = int(tvec[0][0][2]*1000)
		    if distance<=105:
                        self.twist.linear.x=0
                        self.twist.angular.z=0
                        self.cmd_vel_pub.publish(self.twist)
                        self.stop_flag=True
                        print "distance(mm):", distance
                        time.sleep(10) 
		    #self.stop_flag=False

                cv2.imshow("window", image)
                cv2.imshow("window2",bev_img)
                cv2.waitKey(1)

rospy.init_node('lane_follower')
follower = Follower()
rospy.spin()
