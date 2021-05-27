#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from PubSubPy.srv import *

def add_two_ints_client(x, y, z, a, b, c, d):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y, z, a, b, c, d)
        return resp1.V_p
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y z a b c d]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 8:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        a = float(sys.argv[4])
        b = float(sys.argv[5])
        c = float(sys.argv[6])
        d = float(sys.argv[7])
        
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s, %s, %s, %s, %s, %s, %s"%(x, y, z, a, b, c, d))
    print("%s, %s, %s, %s, %s, %s, %s: %s"%(x, y, z, a, b, c, d, add_two_ints_client(x, y, z, a, b, c, d)))