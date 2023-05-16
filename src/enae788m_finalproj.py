#! /usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

PLANT_DETECTED = 1
WEED_DETECTED = -1
NONE_DETECTED = 0

current_state = State()
pose = PositionTarget()
drone_pose = PoseStamped()

# image_pub = rospy.Publisher("hires_processed", Image, queue_size=10)
bridge = CvBridge()
lower_weed = np.array([0, 126, 81])
upper_weed = np.array([80, 255, 255])
lower_plant = np.array([31, 67, 92])
upper_plant = np.array([51, 87, 172])

search_grid = np.zeros((8,5), dtype=int)
# search_grid = [
#         [ 0, 0, 0, 0, 0 ],
#         [ 0, 0, 0, 0, 0 ],
#         [ 0, 0, 0, 0, 0 ],
#         [ 0, 0, 0, 0, 0 ],
#         [ 0, 0, 0, 0, 0 ],
#         [ 0, 0, 0, 0, 0 ],
#         [ 0, 0, 0, 0, 0 ],
#         [ 0, 0, 0, 0, 0 ]]

def state_cb(msg):
    global current_state
    current_state = msg

def pose_cb(msg):
    global drone_pose
    drone_pose = msg

def feet2meters(feet):
    return 0.3048*feet

def image_callback(msg):
    global drone_feed
    drone_feed = bridge.imgmsg_to_cv2(msg, "bgr8")
    rospy.loginfo("Image CB")

def detect_crop(img):
    im = img.copy()
    blank_image = np.zeros((720, 1280), dtype = np.uint8)
    blank_image[550:1280, 380:800] = 255 
    im = cv2.bitwise_and(im, im, mask=blank_image)
    img_hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(img_hsv, lower_weed, upper_weed)
    mask2 = cv2.inRange(img_hsv, lower_plant, upper_plant)
    mask = mask1 + mask2
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel=np.ones((3, 3), np.uint8), iterations=1)
    mask = cv2.GaussianBlur(mask, (3, 3), cv2.BORDER_DEFAULT)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel=np.ones((3, 3), np.uint8), iterations=1)

    new_image = cv2.bitwise_and(img, img, mask = mask)

    contours, ___ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for c in contours:
        area_cnt = cv2.contourArea(c)
        M = cv2.moments(c)
        if M['m00'] != 0.0:
            x_m = int(M['m10']/M['m00'])
            y_m = int(M['m01']/M['m00'])

        if area_cnt > 60:
            x, y, w, h = cv2.boundingRect(c)

            hsv = img_hsv[int(y_m),int(x_m)]
            if (hsv[0]>=31 and hsv[0]<=51):
                rospy.loginfo("Plant Detected")
                return PLANT_DETECTED
            elif(hsv[0]>=0 and hsv[0]<=12):
                rospy.loginfo("Weed Detected")
                return WEED_DETECTED
            else:
                rospy.loginfo("No Crop Detected")
                return NONE_DETECTED            
            cv2.circle(img, (int(x+(w/2)), int(y+(h/2))), int(2), (255,255,255), 2)
            new_image = cv2.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.imwrite("img_"+str(int(y_m)), new_image)

            # image_pub.publish(bridge.cv2_to_imgmsg(new_image, "bgr8"))


if __name__ == "__main__":
    rospy.init_node("offboard_hw5")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)
    front_cam_sub = rospy.Subscriber("/hires", Image, callback = image_callback)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    local_pos_pub_mavros = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=5)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose.position.x = 0
    pose.position.y = 0
    pose.position.z = 0.6
    pose.type_mask = pose.IGNORE_VX | pose.IGNORE_VY | pose.IGNORE_VZ | pose.IGNORE_AFZ | pose.IGNORE_AFY | pose.IGNORE_AFX
    pose.coordinate_frame = pose.FRAME_LOCAL_NED
    pose.yaw = 3.141592/2

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub_mavros.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    waypoints = [[0, 0, 3.141592/2],
                 [0, 4.0, 3.141592/2],
                 [0, 8.0, 3.141592/2],
                 [0, 12.0, 3.141592/2],
                 [0, 14.0, 3.141592/2], 
                 [0, 14.0, 0.0], 
                 [4.0, 14.0, 0.0]]
                #  [4.0, 14.0, -3.141592/2],
                #  [4.0, 10.0, -3.141592/2],
                #  [4.0, 6.0, -3.141592/2],
                #  [4.0, 2.0, -3.141592/2],
                #  [4.0, 0.0, -3.141592/2],
                #  [4.0, 0.0, 0.0],
                #  [8.0, 0.0, 0.0],
                #  [8.0, 0.0, 3.141592/2],
                #  [8.0, 4.0, 3.141592/2],
                #  [8.0, 8.0, 3.141592/2],
                #  [8.0, 12.0, 3.141592/2],
                #  [8.0, 14.0, 3.141592/2]]

    count = 0
    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD"):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            # last_req = rospy.Time.now[)
        else:
            if(not current_state.armed):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                # last_req = rospy.Time.now[)

        for point in waypoints:
            while count < 300:
                if (count == 0):
                    rospy.loginfo("Drone Pose: (%f, %f, %f)", point[0], point[1], point[2])
                pose.position.x = feet2meters(point[0])
                pose.position.y = feet2meters(point[1])
                pose.position.z = 0.6
                pose.yaw = point[2]
                local_pos_pub_mavros.publish(pose)

                count += 1
                rate.sleep()
        
            count = 0
            is_crop = detect_crop(drone_feed)
            # Update 2D grid of plants

