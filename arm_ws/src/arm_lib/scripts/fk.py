from __future__ import print_function
 
from arm_lib.srv import Fk,FkResponse
import rospy
import numpy as np

def rotationX(x):
    thetaX = np.radians(x)
    rotX = np.array([
        [1, 0, 0, 0],
        [0, np.cos(thetaX), -np.sin(thetaX), 0],
        [0, np.sin(thetaX), np.cos(thetaX), 0], 
        [0, 0, 0, 1]
    ], dtype=np.float16)
    return rotX

def rotationY(y):
    thetaY = np.radians(y)
    rotY = np.array([
        [np.cos(thetaY), 0, np.sin(thetaY)],
        [0, 1, 0],
        [-np.sin(thetaY), 0, np.cos(thetaY)],
        [0, 0, 0, 1]

    ])
    return rotY

def rotationZ(z): 
    thetaZ = np.radians(z)
    rotZ = np.array([
        [np.cos(thetaZ), -np.sin(thetaZ), 0, 0],
        [np.sin(thetaZ), np.cos(thetaZ), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    return rotZ

def translationZ(a = 0, b = 0, c = 0):
    traZ = np.array([
        [1, 0, 0, a],
        [0, 1, 0, b],
        [0, 0, 1, c],
        [0, 0, 0, 1],
    ])
    return traZ

def handle_fk(req):
    print("Returning fk...")
    M1 = translationZ(0, 0, 0.1)
    M2 = rotationZ(req.arr_jointPos[0]).dot(translationZ(0, 0, req.arm_dim[0]))
    M3 = rotationX(req.arr_jointPos[1]).dot(translationZ(0, 0, req.arm_dim[1]))
    M4 = rotationX(req.arr_jointPos[2]).dot(translationZ(0, 0, req.arm_dim[2]))
    M5 = rotationX(req.arr_jointPos[3]).dot(translationZ(0, 0, req.arm_dim[3]))
    M6 = rotationZ(req.arr_jointPos[4]).dot(translationZ(0, 0, req.arm_dim[4]))
    M7 = rotationY(req.arr_jointPos[5]).dot(translationZ(0, 0, req.arm_dim[5]))

    M = (((((M1.dot(M2)).dot(M3)).dot(M4)).dot(M5)).dot(M6)).dot(M7)
    result = [M[0][3], M[1][3], M[2][3]]
    return FkResponse(result)

def fk_server():
    rospy.init_node('fk_server')
    s = rospy.Service('fk', Fk, handle_fk)
    print("Forward Kinematics...")
    rospy.spin()
 
if __name__ == "__main__":
    fk_server()