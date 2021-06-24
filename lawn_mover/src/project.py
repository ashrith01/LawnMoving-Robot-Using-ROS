#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

pub = None
active = False
current_pose = Point()
current_yaw = 0
current_state = 0
points = None
count = 0
initial_position = Point()
desired_position = Point()
current_pose_yaw = math.pi / 90 
distance_precision = 0.1
scan_regions = None
count_current_state_time = 0 
count_loop = 0

def change_state(state):
    global current_state
    global srv_client_wall_follower_
    global count_current_state_time
    count_current_state_time = 0
    current_state = state
    if current_state == 2:
        resp = srv_client_wall_follower_(True)
    if current_state == 0:
        resp = srv_client_wall_follower_(False)

# service callbacks
def go_to_point_switch(req):
    global active
    active = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

def done():
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

# callbacks
def clbk_odom(msg):
    global current_pose, current_yaw
    current_pose = msg.pose.pose.position
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    current_yaw = euler[2]

def clbk_laser(msg):
    global scan_regions
    scan_regions = {
        'right':  min(min(msg.ranges[0:143]), 5),
        'fright': min(min(msg.ranges[144:287]), 5),
        'front':  min(min(msg.ranges[288:431]), 5),
        'fleft':  min(min(msg.ranges[432:575]), 5),
        'left':   min(min(msg.ranges[576:719]), 5),
    }

def distance_current_position_to_line(p0):
    global initial_position, desired_position, count, points
    p1 = initial_position
    p2 = desired_position
    up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
    lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
    distance = up_eq / lo_eq
    return distance

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):
    global current_yaw, pub, current_pose_yaw, current_state
    desired_yaw = math.atan2(des_pos.y - current_pose.y, des_pos.x - current_pose.x)
    err_yaw = normalize_angle(desired_yaw - current_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > current_pose_yaw:
        twist_msg.angular.z = 0.7 if err_yaw > 0 else -0.7
    pub.publish(twist_msg)
    if math.fabs(err_yaw) <= current_pose_yaw:
        change_state(1)

def go_straight_ahead(des_pos):
    global current_yaw, pub, current_pose_yaw, current_state, points, count, desired_position, initial_position
    desired_yaw = math.atan2(des_pos.y - current_pose.y, des_pos.x - current_pose.x)
    err_yaw = desired_yaw - current_yaw
    err_pos = math.sqrt(pow(des_pos.y - current_pose.y, 2) + pow(des_pos.x - current_pose.x, 2))
    if err_pos > distance_precision:
        twist_msg = Twist()
        twist_msg.linear.x = 0.2
        twist_msg.angular.z = 0.2 if err_yaw > 0 else -0.2
        pub.publish(twist_msg)
    else:
        if count == len(points):
            change_state(3)
        else:
            count = count + 1
            initial_position.x = points[count-1][0]
            initial_position.y = points[count-1][1]
            initial_position.z = 0
            if count == len(points):
                change_state(3)
            else:
                desired_position.x = points[count][0]
                desired_position.y = points[count][1]
                desired_position.z = 0
                print('Next goal is : ',desired_position.x,' ',desired_position.y)
                change_state(0)
    if math.fabs(err_yaw) > current_pose_yaw:
        change_state(0)

def goals(length,width,laps):
    l,b,n = length,width,laps
    l1 = []
    i,j = 1,1
    k = 1
    for i in range(1,l):
        if(k == 1):
            l1.append([i,1])
            l1.append([i,b-1])
        else:
            l1.append([i,b-1])
            l1.append([i,1])
        k *= -1 
    l2 = []
    for i in range(n):
        for j in l1:
            l2.append(j)
        l1 = l1[::-1]
    l3 = [[1,1]]
    for i in range(1,len(l2)):
        if(l2[i] != l2[i-1]):
            l3.append(l2[i])
    return l3

def main():
    global pub, active, points, count, scan_regions, initial_position, desired_position
    global srv_client_wall_follower_
    global count_current_state_time, count_loop
    
    rospy.init_node('go_to_point')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    sub_laser = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.wait_for_service('/wall_follower_switch')
    srv_client_wall_follower_ = rospy.ServiceProxy('/wall_follower_switch', SetBool)
    srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)
    rate = rospy.Rate(20)

    length=int(input('Enter Length of lawn : '))
    width=int(input('Enter Width of lawn : '))
    laps=int(input('Enter number of laps : '))
    points=goals(length,width,laps)

    initial_position.x = 0
    initial_position.y = 0
    initial_position.z = 0
    desired_position.x = points[0][0]
    desired_position.y = points[0][1]
    desired_position.z = 0
    count = 0

    while not rospy.is_shutdown():
        if scan_regions == None:
            continue

        if count != len(points):
            distance_position_to_line = distance_current_position_to_line(current_pose)

        if scan_regions['front'] > 0.15 and scan_regions['front'] < 1:
            change_state(2)
        if current_state == 0 or current_state == 1:
            if scan_regions['front'] > 0.15 and scan_regions['front'] < 1:
                change_state(2)
        if current_state == 0:
            fix_yaw(desired_position)
        elif current_state == 1:
            go_straight_ahead(desired_position)
        elif current_state == 2:
            if count_current_state_time > 5 and \
               distance_position_to_line < 0.1:
                change_state(0)
                fix_yaw(desired_position)
        elif current_state == 3:
            done()
            print('Process Finished')
            break
        else:
            rospy.logerr('Unknown state!')

        count_loop = count_loop + 1
        if count_loop == 20:
            count_current_state_time = count_current_state_time + 1
            count_loop = 0

if __name__ == '__main__':
    main()