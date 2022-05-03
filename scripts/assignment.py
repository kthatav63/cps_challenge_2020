#!/usr/bin/python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
import math
from tf.transformations import quaternion_from_euler
import time

rock = [60.208, -12.502, 18.775]

radius = 2.0
service_timeout = 30
current_state = False
current_mode = "MANUAL"
last_request = 5
i = 0
angle = 0.0
position_x, position_y, position_z = 0, 0, 0


def state_cb(data):
    global current_mode, current_state
    current_mode = data.mode
    current_state = data.armed


def rover_pose_cb(data):
    global position_x, position_y, position_z
    position_x = data.pose.position.x
    position_y = data.pose.position.y
    position_z = data.pose.position.z


def calculate_heading(rock_pos_x, rock_pos_y):
    global position_x, position_y
    heading = math.degrees(math.atan2((position_y - rock_pos_y), (position_x - rock_pos_x)))
    print("heading = " + str(heading))
    quat = quaternion_from_euler(0.0, 0.0, heading)
    return quat


if __name__ == '__main__':
    global current_state, current_mode
    rospy.init_node("something", anonymous=True)
    rospy.Subscriber("/mavros/state", State, state_cb)
    position = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped,
                     callback=rover_pose_cb)
    try:
        rospy.wait_for_service('/mavros/cmd/arming', service_timeout)
        rospy.wait_for_service('/mavros/set_mode', service_timeout)
        rospy.loginfo('Services are connected and ready')
    except rospy.ROSException as e:
        rospy.logerr('Failed to initialize service')
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    rate = rospy.Rate(20.0)
    while (not rospy.is_shutdown()) and (current_state):
        rospy.spin()
        rate.sleep()
        print("hi")
    print("reached")
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 1
    q = quaternion_from_euler(0.0, 0.0, angle)
    while (not rospy.is_shutdown()) and (i < 100):
        position.publish(pose)
        rate.sleep()
        i = i + 1
    offb_set_mode = "OFFBOARD"
    while not rospy.is_shutdown():
        if current_mode != "OFFBOARD":
            mode_change_response = set_mode_client.call(0, offb_set_mode)
            if not mode_change_response.mode_sent:
                rospy.loginfo("mode change failed")
        if not current_state:
            arm_response = arming_client.call(True)
            if arm_response:
                rospy.loginfo("armed")
            else:
                rospy.loginfo("failed")
        rate = rospy.Rate(10)
        pose.pose.position.x = rock[0] + radius + 1.0
        pose.pose.position.y = rock[1] + radius + 1.0
        pose.pose.position.z = rock[2] + 0.5
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        position.publish(pose)

        if position_z >= rock[2] and position_x >= rock[0]:
            count = 0
            while True:
                count = count + 1
                print(count)
                for i in range(0, 360):
                    pose.pose.position.x = rock[0] + radius * math.cos(math.radians(i))
                    pose.pose.position.y = rock[1] + radius * math.sin(math.radians(i))
                    pose.pose.position.z = rock[2]
                    # qu = calculate_heading(rock[0], rock[1])
                    qu = quaternion_from_euler(0.0, 0.0, math.radians(i - 180))
                    pose.pose.orientation.x = qu[0]
                    pose.pose.orientation.y = qu[1]
                    pose.pose.orientation.z = qu[2]
                    pose.pose.orientation.w = qu[3]
                    position.publish(pose)
                    rate.sleep()
                rock[2] = rock[2] + 0.25
                if count == 5:
                    break
            break