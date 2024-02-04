#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image







rospy.init_node("USB_ROSbridge")
r =  rospy.Rate(5)

pub_frame = rospy.Publisher("USB_camera_pub", Image, queue_size=10)



while not  rospy.is_shutdown():
    vidcap = cv2.VideoCapture(12)

    if vidcap.isOpened():
        ret, frame = vidcap.read()  #capture a frame from live video

        #check whether frame is successfully captured
        if ret:
            # continue to display window until 'q' is pressed
            while(True):
                cv2.imshow("Frame",frame)   #show captured frame
                
                #press 'q' to break out of the loop
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        #print error if frame capturing was unsuccessful
        else:
            print("Error : Failed to capture frame")

    # print error if the connection with camera is unsuccessful
    else:
        print("Cannot open camera")
    
    pub_frame.publish()
    r.sleep()

  





