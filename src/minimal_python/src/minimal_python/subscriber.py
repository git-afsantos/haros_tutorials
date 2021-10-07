# -*- coding: utf-8 -*-

# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

import rospy
from std_msgs.msg import String
from minimal_example.srv import GetCounter, GetCounterResponse

g_counter = 0

def chatter_callback(msg):
    global g_counter
    g_counter += 1

def get_counter(req):
    return GetCounterResponse(g_counter)

def main(args):
    try:
        rospy.init_node('listener')
        sub = rospy.Subscriber('chatter', String, chatter_callback,
                               queue_size=10)
        service = rospy.Service('counter', GetCounter, get_counter)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    return 0

