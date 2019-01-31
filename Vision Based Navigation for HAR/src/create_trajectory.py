#! /usr/bin/env python
# -*- encoding: UTF-8 -*-
import numpy as np


def CreateTraj(DistToHuman, socialDist=2.0):
    """
        Generate a navigation trajectory from Robot to Human
    """
    print("In CreateTrajectory...\n")
    d = DistToHuman/2
    N = 7
    # Waypoints in frame C
    waypoints_x = np.matrix([np.linspace(-d, d, N, endpoint=False)])
    waypoints_x = np.delete(waypoints_x, 0)

    # Define waypoints in frame A
    waypoints = (np.vstack([waypoints_x+d, f(waypoints_x)-f(d)])).T
    waypoints_new = waypoints

    # Alternative method
    # Remove waypoints inside social distance
    # p = np.matrix([DistToHuman, 0])
    # waypoints_new = [w for w in waypoints if np.linalg.norm(w-p)>socialDist]
    # print("waypoints_new: ", waypoints_new)
    # waypoints_new = np.array(waypoints_new)
    # waypoints_new = np.squeeze(waypoints_new, axis=1)

    # Generate previous waypoints array
    prev_waypoints = np.vstack([[[0, 0]], waypoints_new[:-1]])

    # Target position vector in Robot Frame
    target_wp = np.subtract(waypoints_new, prev_waypoints)
    return target_wp

def f(x, a=0.5):
    """
        Trajectory curve equation: y = ax^2
    """
    y = a*np.square(x)
    return y
