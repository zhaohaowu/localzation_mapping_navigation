#!/usr/bin/env python
import math
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped

flag_move = 0

def set_throttle_steer(data):

    global flag_move

    pub_vel_left_rear_wheel = rospy.Publisher('/racecar/left_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_rear_wheel = rospy.Publisher('/racecar/right_rear_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_left_front_wheel = rospy.Publisher('/racecar/left_front_wheel_velocity_controller/command', Float64, queue_size=1)
    pub_vel_right_front_wheel = rospy.Publisher('/racecar/right_front_wheel_velocity_controller/command', Float64, queue_size=1)

    pub_pos_left_steering_hinge = rospy.Publisher('/racecar/left_steering_hinge_position_controller/command', Float64, queue_size=1)
    pub_pos_right_steering_hinge = rospy.Publisher('/racecar/right_steering_hinge_position_controller/command', Float64, queue_size=1)

    throttle = data.drive.speed/0.1
    steer = data.drive.steering_angle

    # luo
    if steer > 0:
        R = 0.325 / math.tan(steer)
        omega = throttle / R
        v_rl = omega * (R - 0.1)
        v_rr = omega * (R + 0.1)
        R1 = math.sqrt(0.325**2 + (R - 0.1)**2)
        R2 = math.sqrt(0.325**2 + (R + 0.1)**2)
        v_fl = omega * R1
        v_fr = omega * R2
        theta_fl = math.atan2(0.325, R - 0.1)
        theta_fr = math.atan2(0.325, R + 0.1)
    elif steer < 0:
        steer *= -1
        R = 0.325 / math.tan(steer)
        omega = throttle / R
        v_rl = omega * (R + 0.1)
        v_rr = omega * (R - 0.1)
        R1 = math.sqrt(0.325**2 + (R + 0.1)**2)
        R2 = math.sqrt(0.325**2 + (R - 0.1)**2)
        v_fl = omega * R1
        v_fr = omega * R2
        theta_fl = -1 * math.atan2(0.325, R + 0.1)
        theta_fr = -1 * math.atan2(0.325, R - 0.1)  
    else:
        theta_fl = theta_fr = 0.0
        v_rl = v_rr = v_fl = v_fr = throttle

    pub_vel_left_rear_wheel.publish(v_rl)
    pub_vel_right_rear_wheel.publish(v_rr)
    pub_vel_left_front_wheel.publish(v_fl)
    pub_vel_right_front_wheel.publish(v_fr)
    pub_pos_left_steering_hinge.publish(theta_fl)
    pub_pos_right_steering_hinge.publish(theta_fr)

    # pub_vel_left_rear_wheel.publish(throttle)
    # pub_vel_right_rear_wheel.publish(throttle)
    # pub_vel_left_front_wheel.publish(throttle)
    # pub_vel_right_front_wheel.publish(throttle)
    # pub_pos_left_steering_hinge.publish(steer)
    # pub_pos_right_steering_hinge.publish(steer)

def servo_commands():

    rospy.init_node('servo_commands', anonymous=True)

    rospy.Subscriber("/ackermann_cmd_mux/output", AckermannDriveStamped, set_throttle_steer)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
