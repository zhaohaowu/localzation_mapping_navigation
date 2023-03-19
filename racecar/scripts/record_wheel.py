#!/usr/bin/env python
# coding=utf-8

import csv
import math
import rospy
import numpy as np
from std_msgs.msg import Float64
import matplotlib.pyplot as plt

# plt.rc('font', family='Times New Roman')
# plt.rcParams['axes.unicode_minus'] = False

fl_theta = 0
fr_theta = 0
rl_v = 0
rr_v = 0
fl_v = 0
fr_v = 0

def csv_recorder(filename, data):
    with open(filename, 'a+') as f:
        csv.writer(f).writerow([data])

def left_rear_wheel_velocity_controller(data):
    print(1)
    rl_v = data
    csv_recorder('rl_v.csv', data)

def right_rear_wheel_velocity_controller(data):
    rr_v = data
    csv_recorder('rr_v.csv', data)

def left_front_wheel_velocity_controller(data):
    fl_v = data
    csv_recorder('fl_v.csv', data)

def right_front_wheel_velocity_controller(data):
    fr_v = data
    csv_recorder('fr_v.csv', data)

def left_steering_hinge_position_controller(data):
    fl_theta = data
    csv_recorder('fl_theta.csv', data)

def right_steering_hinge_position_controller(data):
    fr_theta = data
    csv_recorder('fr_theta.csv', data)

def kino_compute(v, theta, x, y):
    x = np.append(x, x[-1] + v * math.cos(theta))
    x = np.append(y, y[-1] + v * math.sin(theta))
    # x = x + v * math.cos(theta)
    # y = y + v * math.sin(theta)
    return x, y

def main():
    rospy.init_node('record_wheel', anonymous=True)
    rospy.Subscriber("/racecar/left_rear_wheel_velocity_controller/command", Float64, left_rear_wheel_velocity_controller)
    rospy.Subscriber("/racecar/right_rear_wheel_velocity_controller/command", Float64, right_rear_wheel_velocity_controller)
    rospy.Subscriber("/racecar/left_front_wheel_velocity_controller/command", Float64, left_front_wheel_velocity_controller)
    rospy.Subscriber("/racecar/right_front_wheel_velocity_controller/command", Float64, right_front_wheel_velocity_controller)
    rospy.Subscriber("/racecar/left_steering_hinge_position_controller/command", Float64, left_steering_hinge_position_controller)
    rospy.Subscriber("/racecar/right_steering_hinge_position_controller/command", Float64, right_steering_hinge_position_controller)

    # # 绘图
    # beishu = 0.6
    # parameters = {'axes.labelsize': 40 * beishu,
    # 'axes.titlesize': 45 * beishu,
    # 'figure.titlesize': 40 * beishu,
    # 'xtick.labelsize': 35 * beishu,
    # 'ytick.labelsize': 35 * beishu,
    # 'legend.fontsize': 40 * beishu}
    # plt.rcParams.update(parameters)

    # fig, axs = plt.subplots(nrows=2, ncols=1, figsize=(24, 12))

    # rate = rospy.Rate(10)
    # x_fl = y_fl = x_fr = y_fr = np.array([0.0])
    # while not rospy.is_shutdown():
    #     x_fl, y_fl = kino_compute(fl_v, fl_theta, x_fl, y_fl)
    #     x_fr, y_fr = kino_compute(fr_v, fr_theta, x_fr, y_fr)

    #     axs[0].scatter(x_fl, y_fl)
    #     axs[0].set_title('front left steer')
    #     axs[0].set_xlabel('X')
    #     axs[0].set_ylabel('Y')

    #     axs[1].scatter(x_fr, y_fr)
    #     axs[1].set_title('front right steer')
    #     axs[1].set_xlabel('X')
    #     axs[1].set_ylabel('Y')

    #     plt.ion()
    #     plt.pause(1)
    #     plt.ioff()
    #     # plt.show()

    #     # rospy.spinOnce()
    #     rate.sleep()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    main()