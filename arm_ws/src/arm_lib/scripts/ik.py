from __future__ import print_function

from arm_lib.srv import Ik, IkResponse
import rospy
import numpy as np

import tinyik as ik

arm = ik.Actuator([
    "z", [0, 0, 0.15],
    "x", [0, 0, 2.0],
    "x", [0, 0, 1.0],
    "x", [0, 0, .5],
    "z", [0, 0, 0.02],
    "x", [0, 0, 0.02]
])


def handle_ik(req):
    print(f"Received Positions: {req.xyz_coordinates}")
    print(f"Received Angles {req.arr_jointPos}")
    print(f"Received Links {req.arm_dim}")
    print("Returning IK")
    arm = ik.Actuator([
        "z", [0, 0, req.arm_dim[0]],
        "x", [0, 0, req.arm_dim[1]],
        "x", [0, 0, req.arm_dim[2]],
        "x", [0, 0, req.arm_dim[3]],
        "z", [0, 0, req.arm_dim[4]],
        "y", [0, 0, req.arm_dim[5]]
    ])
    arm.angles = [req.arr_jointPos[0], req.arr_jointPos[1], req.arr_jointPos[2],
                  req.arr_jointPos[3], req.arr_jointPos[4], req.arr_jointPos[5]]
    arm.ee = [req.xyz_coordinates[0],
              req.xyz_coordinates[0], req.xyz_coordinates[0]]
    return IkResponse(arm.angles)


def ik_server():
    rospy.init_node('ik_server')
    s = rospy.Service('ik', Ik, handle_ik)
    print("Inverse Kinematics...")
    rospy.spin()


if __name__ == "__main__":
    ik_server()
