#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic
import rospy
from std_msgs.msg import String
from arl_nav_msgs.msg import GotoRegionActionGoal
from actionlib_msgs.msg import GoalStatusArray
import time

def callback(data):
    global feedback_text
    feedback_text = ""
    if data:
        feedback_text = data.status_list[0].text

def talker(x, y):
    goal_msg = GotoRegionActionGoal()
    goal_msg.goal.region_center.pose.position.x = x
    goal_msg.goal.region_center.pose.position.y = y
    goal_msg.goal.radius = 1.25
    goal_msg.goal.angle_threshold = 1.47
    goal_msg.goal.region_center.header.frame_id = "warty/map"
    rospy.Subscriber("/warty/goto_region/status", GoalStatusArray, callback)
    pub = rospy.Publisher('/warty/goto_region/goal', GotoRegionActionGoal, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    for i in range(2):
        pub.publish(goal_msg)
        rate.sleep()
    switch = 0
    while True:
        if feedback_text != "Within radius and theta of goal region" and feedback_text !="Local planner finished":
            switch = 1
        elif switch == 1:
            break

if __name__ == '__main__':
    policy = open("traj_robust_multi-obj-arl.txt", 'r')
    for index, line in enumerate(policy):
        if index < 70:
            continue
        line_split = line.split(" ")
        x = int(line_split[0].strip('(, '))
        y = int(line_split[1].strip(')\n'))
        #x = 178
        #y = 222
        next_y = -473.75 + y*2.5 - 23 - 7.5
        next_x = -198.75 + x*2.5 + 21 + 2.5
        print(next_x,next_y)
        talker(next_x,next_y)

