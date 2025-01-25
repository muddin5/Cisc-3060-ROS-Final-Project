import sys
import rospy
from geometry_msgs.msg import Twist      # ROS Twist message
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import random

# ROS topic names for turtlebot_gazebo
safe_cmd_topic = '/safe_cmd_vel'
motionTopic ='/cmd_vel' 
imageTopic = '/camera/rgb/image_raw'
laserTopic = '/scan'

# Global variables
gCurrentImage = Image() # make a global variable for the image
gBridge = CvBridge()    # make a ROS to CV bridge object
gImageStarted=False

obstacle_left = False
obstacle_right = False



# Callback for the image topic--------------------------------------------
def callbackImage(img):
    '''Called automatically for each new image'''
    global gCurrentImage, gBridge, gImageStarted
    gCurrentImage = gBridge.imgmsg_to_cv2(img, "bgr8")
    gImageStarted=True
    return
# procedure to display whatever images are present

    
#--------------MULTIPLE Color tracking----------------------------------------------
def trackNode(targetCols):
    '''center the robot camera on the target if in view'''
    global gCurrentImage

    rospy.init_node('displayNode',anonymous=True)
    # create windows to show camera and processing
    cv2.namedWindow('Turtlebot Camera', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('Target', cv2.WINDOW_AUTOSIZE)

    imageSub = rospy.Subscriber(imageTopic,Image,callbackImage)
    vel_pub = rospy.Publisher(motionTopic, Twist, queue_size=0)
    rospy.sleep(2) # wait for callbacks to catch up
    while not gImageStarted:
        rospy.sleep(1) # wait for callback to catch up

    rate = rospy.Rate(10)
    msg=Twist()
    start = rospy.get_time()
    
    tnum = 0;
    for targetCol in targetCols:
    	print(f"Tracking target color: {targetCol[0]}")
        tnum = tnum+1
        
        
        spinCycle = 200
    	CycleCount = 0
    	while not rospy.is_shutdown():
    	    #just show what the camera sees now
    	    cv2.imshow('Turtlebot Camera', cv2.resize(gCurrentImage,(320,240)))
    	    #get height h and width w of current image
    	    h,w = gCurrentImage.shape[0], gCurrentImage.shape[1]

    	    # make a binary image that is 0 except where the color is in range
    	    targetImage = cv2.inRange(gCurrentImage,targetCol[1],targetCol[2])
    	    cv2.imshow('Target',cv2.resize(targetImage,(320,240)))
    	    
    	    found = False
    	    avel = 0.0
    	    Lvel = 0.0
       
    	    #tracking algorithm
    	    if not found:
    	    	if CycleCount < spinCycle and CycleCount >= 0:
    	    		avel=0.4 # default velocity, so robot 'spins' when no target in view
    	    		Lvel = 0.0
    	    		print(f"Spinning. Searching for {targetCol[0]} ", CycleCount)
    	    		CycleCount = CycleCount + 1
    	    		vel_pub.publish(msg)
    	    	
    	    	elif CycleCount < 0:
    	    		avel= random.uniform(0.2, 0.5)  # Random forward speed
    	    		Lvel = random.uniform(-0.5, 0.8)
    	    		print(f"wandering, searcing for: ", targetCol[0], " ",CycleCount)    	   
    	    		CycleCount = CycleCount + 1
    	    		vel_pub.publish(msg)
    	    	else:
    	    		CycleCount = -80
    	    
    	    	vel_pub.publish(msg)
    	    
    	    	
    	    # extract the moments of the target image
    	    m = cv2.moments(targetImage)
    	    
    	    
    	    if m['m00']> 1000000: # skip if the target image has non nonzero regions
    	        # how far is the X center of target  from X center of image
    	        delx = w/2 - m['m10']/m['m00']
    	        dely = h*0.6 - m['m01'] / m['m00']
    	        avel = 0.5*delx/w # use this to generate a proportional ang velocity
    	        Lvel = 0.8 * dely/h
    	        min_dist = 310
    	        max_dist = 340
    	        if dely < 0:
    	           Lvel = -0.8 * dely/h
    	        elif dely < min_dist:
    	            0.8 * dely/h
    	        elif dely > max_dist:
    	            Lvel = -0.8*dely/h     	               	            
    	        else:
    	            if min_dist < dely < max_dist:
    	            	found = True
    	
    	        print ("Target center=",
    	               round(m['m10']/m['m00'],2),round(m['m01']/m['m00'],2),end=" ")
    	        print ("Img center offset=",round(delx,2),
    	               " => avel=",round(avel,2), "Lvel=",round(Lvel, 6))
    	        print ("dely=", dely, "       ", targetCol[0])
    	    msg.linear.x, msg.angular.z = Lvel, avel # publish velocity
    	    vel_pub.publish(msg)
    	    cv2.waitKey(1) # need to do this for CV2 windows to show up
    	    rate.sleep()
    	    
    	    if found == True:
    	    	print(f"Reached target for color range: {targetCol}")
    	    	cycleCount = 0
    	    	elapsed = rospy.get_time() - start
    	    	txt1 = "Tracking Time: " +str(elapsed)
    	    	txt2 = "Goal "+str(tnum)+": "+targetCol[0]
    	    	font = cv2.FONT_HERSHEY_PLAIN
    	    	red = (0, 0, 255)
    	    	cv2.putText(gCurrentImage, txt1, (0, 100), font, 2, red)
    	    	fileName = targetCol[0]+".jpg"
    	    	cv2.imwrite(fileName, gCurrentImage)
    	    	#make a jPeg Buddy
    	    	break
    	    
    return
    
#------------------OBSTACLE AVOIDANCE NODE—-------------------------------------------------------------
def callback_laser(msg):
    '''
    callback for laser scan data
    this finds the index corresponding to the robot's forward direction,
    then checks sectors to the left and right of that direction for obstacles
    '''
    global obstacle_left, obstacle_right


    # find the index corresponding to the front (0 degrees)
    zero_index = int((0.0 - msg.angle_min) / msg.angle_increment)


    # determine how many laser samples correspond to about 30 degrees
    deg_per_index = math.degrees(msg.angle_increment)
    indices_per_30_deg = int(30 / deg_per_index)


    # define left and right sectors relative to the front
    # left sector: from zero_index - indices_per_30_deg up to zero_index
    left_start = max(zero_index - indices_per_30_deg, 0)
    left_end = zero_index
    


    # right sector: from zero_index to zero_index + indices_per_30_deg
    right_start = zero_index
    right_end = min(zero_index + indices_per_30_deg, len(msg.ranges) - 1)


    # reset obstacle flags
    obstacle_left = False
    obstacle_right = False

    too_close = 1.5  # threshold for obstacle detection

    # check the left sector for obstacles
    for i in range(left_start, left_end):
        dist = msg.ranges[i]
        if not math.isnan(dist) and not math.isinf(dist) and dist < too_close:
            obstacle_left = True
            break


    # check the right sector for obstacles
    for i in range(right_start, right_end):
        dist = msg.ranges[i]
        if not math.isnan(dist) and not math.isinf(dist) and dist < too_close:
            obstacle_right = True
            break
            
def cmd_callback(msg):
    '''
    callback for /safe_cmd_vel commands
    applies obstacle avoidance if needed before publishing to /cmd_vel
    '''
    global obstacle_left, obstacle_right


    # start by passing the message through unchanged
    safe_msg = Twist()
    safe_msg.linear.x = msg.linear.x
    safe_msg.angular.z = msg.angular.z


    # obstacle avoidance only if moving forward
    if msg.linear.x > 0.0:
        if obstacle_left and not obstacle_right:
            # obstacle on left side, turn right and slow down
            print(f"Turning Right")
            safe_msg.linear.x = 0.5 * msg.linear.x
            safe_msg.angular.z = -0.2
        elif obstacle_right and not obstacle_left:
            # obstacle on right side, turn left and slow down
            print(f"Turning Left")
            safe_msg.linear.x = 0.5 * msg.linear.x
            safe_msg.angular.z = 0.2
        elif obstacle_left and obstacle_right:
            # obstacles on both sides
            # stop and turn slightly to try avoiding them
            print(f"Moving Back")
            safe_msg.linear.x = -0.2
            safe_msg.angular.z = 0.3


    pub_cmd.publish(safe_msg)


#-----------------------------------------------------------------------    
def callback_shutdown():
    print("Shutting down")
    pub = rospy.Publisher(motionTopic, Twist, queue_size=1)
    msg = Twist()
    msg.angular.z=0.0
    msg.linear.x=0.0
    pub.publish(msg) 
    rospy.sleep(5)
    return   
    
#--------------What's actually running---------------------------------
if __name__ == '__main__':
    # identify/center the RGB color range of the target
    try:
        rospy.init_node('displayNode', anonymous = True)
        rospy.on_shutdown(callback_shutdown)
        targetColors = [
        ["Orange", (0,30,75), (5,50,89)],
        ["White", (240, 240, 240), (255, 255, 255)],
        ["Sand", (0, 80, 80), (10, 110, 110)], 
        ["Black", (0, 0, 0),(20, 20, 20)]
        ]
        trackNode(targetColors)
    except  rospy.ROSInterruptException:
        pass
#
