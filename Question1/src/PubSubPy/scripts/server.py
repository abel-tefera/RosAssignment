#!/usr/bin/env python

from __future__ import print_function

from PubSubPy.srv import AddTwoInts,AddTwoIntsResponse
import rospy
import numpy as np

def handle_add_two_ints(req):
    print("Returning [%s, %s, %s, %s, %s, %s, %s]"%(req.Vx, req.Vy, req.Vz, req.Ra, req.Rb, req.Rc, req.Td))
    V = np.array([req.Vx, req.Vy, req.Vz])
    return handle_rotation(req.Ra, req.Rb, req.Rc, V, req.Td)

def handle_rotation(x, y, z, V, d):
    thetaX = np.degrees(x)
    thetaY = np.degrees(y)
    thetaZ = np.degrees(z)

    rotX = np.array([
        [1, 0, 0],
        [0, np.cos(thetaX), -np.sin(thetaX)],
        [0, np.sin(thetaX), np.cos(thetaX)]
    ])
    rotY = np.array([
        [np.cos(thetaY), 0, np.sin(thetaY)],
        [0, 1, 0],
        [-np.sin(thetaY), 0, np.cos(thetaY)]
    ])
    rotZ = np.array([
        [np.cos(thetaZ), -np.sin(thetaZ), 0],
        [np.sin(thetaZ), np.cos(thetaZ), 0],
        [0, 0, 1],
    ])

    Rx = V.dot(rotX)
    Ry = Rx.dot(rotY)
    Rz = Ry.dot(rotZ)
    Rz[0] += d
    Rz[1] += d
    Rz[2] += d

    # print(V, Rx, Ry, Rz)
    listToStr = ' '.join([str(elem) for elem in Rz])
    return AddTwoIntsResponse(listToStr)


def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to rotate and translate a vector.")
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()