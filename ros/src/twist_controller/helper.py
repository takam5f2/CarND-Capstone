import rospy
from math import sqrt, cos, sin
import numpy as np
import math
import tf


def get_yaw_euler_angle(pose):
    return tf.transformations.euler_from_quaternion([pose.orientation.x,
                                                     pose.orientation.y,
                                                     pose.orientation.z,
                                                     pose.orientation.w])

def is_waypoint_behind(pose, waypoint):
    _, _, yaw = get_euler(pose)
    pose_X = pose.position.x
    pose_Y = pose.position.y

    shift_X = waypoint.pose.pose.position.x - pose_X
    shift_Y = waypoint.pose.pose.position.y - pose_Y

    waypoint_X = shift_x * cos(0-yaw) - shift_Y * sin(0-yaw)
    if waypoint_X > 0:
        return False
    return True

def transform_waypoints_to_ego_point(pose, yaw, waypoints, points_to_use=None):
    coords_X = []
    coords_Y = []

    if yaw is None:
        _, _, yaw = get_yaw_euler_angle(pose)
    ego_X = pose.position.x
    ego_Y = pose.position.y

    if points_to_use is None:
        points_num = len(waypoints)
    else:
        points_num = points_to_use
        
    if points_to_use >= len(waypoints):
        points_num = len(waypoints)
        
    for i in range(points_num):
        try:
            shift_X = waypoints[i].pose.pose.position.x - ego_X
            shift_Y = waypoints[i].pose.pose.position.y - ego_Y
        except IndexError:
            rospy.logwarn("lengh of waypoints = {}, points_to_use = {}, index = {}".format(len(waypoints), points_to_use, i))
            break

        new_x = shift_X * cos(0-yaw) - shift_Y * sin(0-yaw)
        new_y = shift_X * sin(0-yaw) + shift_Y * cos(0-yaw)

        coords_X.append(new_x)
        coords_Y.append(new_y)

    return coords_X, coords_Y


def fit_waypoints(pose, waypoints, yaw=None, polynomial_order=3, points_to_use=10):
    coords_X, coords_Y = transform_waypoints_to_ego_point(pose, yaw, waypoints, points_to_use)
    rospy.logwarn("coords_X: {0}".format(coords_X))
    rospy.logwarn("coords_Y: {0}".format(coords_Y))
    polynomial = np.polyfit(coords_X, coords_Y, polynomial_order)

    return polynomial

def get_cte(pose, waypoints):
    if len(waypoints) == 0 or waypoints is None:
        cte = 0.0
    else:
        polynomial = fit_waypoints(pose, waypoints)
        rospy.logwarn("polynomial coefficient: {0}".format(polynomial))
        cte = np.polyval(polynomial, 5.0)
    return cte
