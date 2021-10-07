# -*- coding: utf-8 -*-

# SPDX-License-Identifier: MIT
# Copyright © 2021 André Santos

import rospy
from std_msgs.msg import String
from minimal_example.srv import GetCounter

def publish_hello(pub):
    msg = String()
    msg.data = 'hello world'
    pub.publish(msg)

def get_counter(client):
    try:
        response = client()
        rospy.loginfo('Counter: {}'.format(response.counter))
    except rospy.ServiceException as e:
        rospy.logerr('Failed to call service counter')
        rospy.logerr(repr(e))

def run_node(freq_param):
    rospy.init_node('talker')
    pub = rospy.Publisher('monologue', String, queue_size=10)
    client = rospy.ServiceProxy('counter', GetCounter)
    frequency = rospy.get_param(freq_param, -1)
    if frequency == -1:
        rospy.logerr('Could not get frequency from parameter.')
        frequency = 10
    else:
        rospy.loginfo('Got frequency from parameter.')
    loop_rate = rospy.Rate(frequency)
    while not rospy.is_shutdown():
        publish_hello(pub)
        get_counter(client)
        loop_rate.sleep()

def main(args):
    if len(args) < 1:
        return 1
    try:
        run_node(args[0])
    except rospy.ROSInterruptException:
        pass
    return 0
    

