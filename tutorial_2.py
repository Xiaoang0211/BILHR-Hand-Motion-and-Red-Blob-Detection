#!/usr/bin/env python

'''
Student names:
Qiwen Xu
Jinjun Dong
Xiaoang Zhang
Tomas Schweizer
Leon Mayer
'''

import rospy
from std_msgs.msg import String
from std_srvs.srv import Empty
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed,Bumper,HeadTouch
from sensor_msgs.msg import Image,JointState
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class Central:

    ####### Task 7


    #### setting home position for arms
    def set_home_position(self,both):

        # setting the left arm to the home position
        self.set_joint_angles(0.4,"LShoulderRoll")

        self.set_joint_angles(1.5,"LShoulderPitch")
        
        self.set_joint_angles(-0.3,"LElbowYaw")
        
        self.set_joint_angles(0.1,"LElbowRoll")
        
        self.set_joint_angles(-1.5,"LWristYaw")

        if both == True:
            # setting the right arm also to the home position
            self.set_joint_angles(-0.4,"RShoulderRoll")

            self.set_joint_angles(1.5,"RShoulderPitch")
            
            self.set_joint_angles(-0.3,"RElbowYaw")
            
            self.set_joint_angles(0.1,"RElbowRoll")
            
            self.set_joint_angles(1.5,"RWristYaw")




        

    #### motion for Right Arm
    def start_motion_R(self):
        self.set_joint_angles(-0.5,"RShoulderPitch")
        rospy.sleep(2)
        self.set_joint_angles(0.8,"RElbowRoll")
        rospy.sleep(1)
        self.set_joint_angles(-1,"RWristYaw")
        rospy.sleep(1)
        self.set_joint_angles(1,"RWristYaw")
        rospy.sleep(1)
        self.set_joint_angles(-1,"RWristYaw")
        rospy.sleep(1)
        self.set_joint_angles(-1,"RWristYaw")
        rospy.sleep(1)
        self.set_joint_angles(-1,"RWristYaw")
        rospy.sleep(1)
        self.set_joint_angles(-1,"RWristYaw")
    
    ### right arm mirrors left arm
    def start_motion_mirror(self):

        self.set_joint_angles(-0.5,"LShoulderPitch")
        self.set_joint_angles(-0.5,"RShoulderPitch")

        rospy.sleep(1)

        self.set_joint_angles(-0.8,"LElbowRoll")
        self.set_joint_angles(0.8,"RElbowRoll")

        rospy.sleep(1)

        self.set_joint_angles(-1,"LWristYaw")
        self.set_joint_angles(1,"RWristYaw")

        rospy.sleep(1)

        self.set_joint_angles(1,"LWristYaw")
        self.set_joint_angles(-1,"RWristYaw")

        rospy.sleep(1)

        self.set_joint_angles(-1,"LWristYaw")
        self.set_joint_angles(1,"RWristYaw")

        rospy.sleep(1)

        self.set_joint_angles(-1,"LWristYaw")
        self.set_joint_angles(1,"RWristYaw")

        rospy.sleep(1)

        self.set_joint_angles(-1,"LWristYaw")
        self.set_joint_angles(1,"RWristYaw")

        rospy.sleep(1)

        self.set_joint_angles(-1,"LWristYaw")
        self.set_joint_angles(1,"RWristYaw")

    #### motion for Left Arm
    def start_motion_L(self):
        self.set_joint_angles(-0.5,"LShoulderPitch")
        rospy.sleep(2)
        self.set_joint_angles(-0.8,"LElbowRoll")
        rospy.sleep(1)
        self.set_joint_angles(-1,"LWristYaw")
        rospy.sleep(1)
        self.set_joint_angles(1,"LWristYaw")
        rospy.sleep(1)
        self.set_joint_angles(-1,"LWristYaw")
        rospy.sleep(1)
        self.set_joint_angles(-1,"LWristYaw")
        rospy.sleep(1)
        self.set_joint_angles(-1,"LWristYaw")
        rospy.sleep(1)
        self.set_joint_angles(-1,"LWristYaw")



    def __init__(self):
        # initialize class variables
        self.joint_names = []
        self.joint_angles = []
        self.joint_velocities = []
        self.jointPub = 0
        self.stiffness = False  
        self.motion_state = 2 # 0: Home, 1: Motion, 2: Both Arms
        pass


    def key_cb(self,data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    def joints_cb(self,data):
        #rospy.loginfo("joint states "+str(data.name)+str(data.position))
        # store current joint information in class variables
        self.joint_names = data.name 
        self.joint_angles = data.position
        self.joint_velocities = data.velocity

        pass

    def bumper_cb(self,data):
        rospy.loginfo("bumper: "+str(data.bumper)+" state: "+str(data.state))
        if data.bumper == 0:
            self.stiffness = True
        elif data.bumper == 1:
            self.stiffness = False

    ###sets motion_state and logs infos
    def touch_cb(self,data):
        rospy.loginfo("touch button: "+str(data.button)+" state: "+str(data.state))
    #####
        rospy.loginfo("button clicked!")
        if data.button == 1:
            rospy.loginfo("button 1 clicked!")
            self.motion_state = 1

        elif data.button == 2:
            rospy.loginfo("button 2 clicked!")
            self.motion_state = 2

        elif data.button == 3:
            rospy.loginfo("button 3 clicked!")
            self.motion_state = 3   
            

    ##### Task 9: red blob capturing

    ###performs red blob detection with contours
    def image_cb(self,data):
        bridge_instance = CvBridge()
        try:
            cv_image = bridge_instance.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        ### task 9
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        #sets color range to detect shades of red
        low_red1 = np.array([0, 100, 20])
        high_red1 = np.array([10, 255, 255])
        low_red2 = np.array([160, 100, 20])
        high_red2 = np.array([179, 255, 255])

        #creates masks with red regions
        lower_red_mask = cv2.inRange(hsv, low_red1, high_red1)
        upper_red_mask = cv2.inRange(hsv, low_red2, high_red2)
        full_mask = lower_red_mask + upper_red_mask
        
        #cuts out the red areas
        red = cv2.bitwise_and(cv_image, cv_image, mask = full_mask)

        #converts red mask to gray
        gray = cv2.cvtColor(red, cv2.COLOR_BGR2GRAY)
        
        #turns image into black and white
        thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 12)
        cv2.imshow('Test0', thresh)

        #morphological transformation to get nice shaped blob, which we don't
        #actually not used here because the detected objects are also not perfectly round-shaped blob
        # get the desired structuring element, well ellipse (blob)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9,9))
        blob = cv2.morphologyEx(blob, cv2.MORPH_CLOSE, kernel) # erosion followed by dilation,uselful in removing noise

	# inverse the black and white
        blob = (255 - blob)

        # inverse the black and white
        thresh = (255 - thresh)
        cv2.imshow('Test1', thresh)

        # Get contours
        cnts = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1] 
        # get the largest contour
        big_contour = max(cnts, key=cv2.contourArea)

        

        # draw contour
        result = cv_image.copy()
        cv2.drawContours(result, [big_contour], -1, (0,255,0), 1)
        #extract center coordinates
        M = cv2.moments(big_contour)
        cX = float(M["m10"]) / float(M["m00"])
        cY = float(M["m01"]) / float(M["m00"])
        #draws circle at center coordinates
        cv2.circle(result, (int(cX), int(cY)), 7, (0, 255, 0), -1)
        #cv2.imshow("Red", red)
        cv2.imshow('Test2', result)
        #logs center coordinates
        rospy.loginfo("X: %f \t Y: %f", int(cX), int(cY))

        cv2.waitKey(3) # a small wait time is needed for the image to be displayed correctly



    # sets the stiffness for all joints. can be refined to only toggle single joints, set values between [0,1] etc
    def set_stiffness(self,value):
        if value == True:
            service_name = '/body_stiffness/enable'
        elif value == False:
            service_name = '/body_stiffness/disable'
        try:
            stiffness_service = rospy.ServiceProxy(service_name,Empty)
            stiffness_service()
        except rospy.ServiceException, e:
            rospy.logerr(e)

    def set_joint_angles(self,head_angle,joint_name):

        joint_angles_to_set = JointAnglesWithSpeed()
        
        joint_angles_to_set.joint_names.append(joint_name) # each joint has a specific name, look into the joint_state topic or google
        joint_angles_to_set.joint_angles.append(head_angle) # the joint values have to be in the same order as the names!!
        joint_angles_to_set.relative = False # if true you can increment positions
        joint_angles_to_set.speed = 0.1 # keep this low if you can
        self.jointPub.publish(joint_angles_to_set)
        


    def central_execute(self):
        rospy.init_node('central_node',anonymous=True) #initilizes node, sets name

        # create several topic subscribers
        rospy.Subscriber("key", String, self.key_cb)
        rospy.Subscriber("joint_states",JointState,self.joints_cb)
        rospy.Subscriber("bumper",Bumper,self.bumper_cb)
        rospy.Subscriber("tactile_touch",HeadTouch,self.touch_cb)
        rospy.Subscriber("/nao_robot/camera/top/camera/image_raw",Image,self.image_cb)
        self.jointPub = rospy.Publisher("joint_angles",JointAnglesWithSpeed,queue_size=25)


        # test sequence to demonstrate setting joint angles
        self.set_stiffness(True) # don't let the robot stay enabled for too long, the motors will overheat!! (don't go for lunch or something)
        self.stiffness = True
        rospy.sleep(1.0)
        self.set_home_position(True) #set robot to home position when started
        rospy.sleep(3.0)

        rate = rospy.Rate(10) # sets the sleep time to 10ms
    
        #start arm motion routine
        while not rospy.is_shutdown():
            self.set_stiffness(self.stiffness)
            if self.motion_state == 2: # button 2: left arm repitetive
                self.start_motion_L()
                rospy.sleep(2)
                self.set_home_position(True)
                rospy.sleep(3)
            elif self.motion_state == 1: # button 1: left arm to home position
                self.set_home_position(False)

            elif self.motion_state == 3: # button 3: simutaneously moving the right arm mirroring the left arm
                self.start_motion_mirror()
                rospy.sleep(2)
                self.set_home_position(True)
                rospy.sleep(3)
            
            rate.sleep()

    # rospy.spin() just blocks the code from exiting, if you need to do any periodic tasks use the above loop
    # each Subscriber is handled in its own thread
    #rospy.spin()

if __name__=='__main__':
    # instantiate class and start loop function
    central_instance = Central()
    central_instance.central_execute()
    
