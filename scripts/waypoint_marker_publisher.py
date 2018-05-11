#!/usr/bin/env python
# /****************************************************************************
#  *   Copyright (c) 2018 John A. Dougherty. All rights reserved.
#  *
#  * Redistribution and use in source and binary forms, with or without
#  * modification, are permitted provided that the following conditions
#  * are met:
#  *
#  * 1. Redistributions of source code must retain the above copyright
#  *    notice, this list of conditions and the following disclaimer.
#  * 2. Redistributions in binary form must reproduce the above copyright
#  *    notice, this list of conditions and the following disclaimer in
#  *    the documentation and/or other materials provided with the
#  *    distribution.
#  * 3. Neither the name ATLFlight nor the names of its contributors may be
#  *    used to endorse or promote products derived from this software
#  *    without specific prior written permission.
#  *
#  * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
#  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
#  * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#  * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  * POSSIBILITY OF SUCH DAMAGE.
#  *
#  * In addition Supplemental Terms apply.  See the SUPPLEMENTAL file.
#  ****************************************************************************/

import rospy

from geometry_msgs.msg import Point
from snav_msgs.msg import WaypointArray
from visualization_msgs.msg import Marker

pub = rospy.Publisher('planner/waypoints/markers', Marker, queue_size=10)

def input_waypoint_callback(data):
  msg = Marker()
  msg.header = data.header
  msg.ns = "input"
  msg.lifetime = rospy.Duration(1.0)
  msg.color.r = 0
  msg.color.g = 0.5
  msg.color.b = 1.0
  msg.color.a = 0.8
  msg.type = Marker.SPHERE_LIST
  msg.scale.x = 0.1
  msg.scale.y = 0.1
  msg.scale.z = 0.1
  for wp in data.waypoints:
    pt = Point(x=wp.position.x, y=wp.position.y, z=wp.position.z)
    msg.points.append(pt)
  pub.publish(msg)

def opt_waypoint_callback(data):
  msg = Marker()
  msg.header = data.header
  msg.ns = "optimized"
  msg.lifetime = rospy.Duration(1.0)
  msg.color.r = 0
  msg.color.g = 1.0
  msg.color.b = 0.5
  msg.color.a = 0.8
  msg.type = Marker.SPHERE_LIST
  msg.scale.x = 0.05
  msg.scale.y = 0.05
  msg.scale.z = 0.05
  for wp in data.waypoints:
    pt = Point(x=wp.position.x, y=wp.position.y, z=wp.position.z)
    msg.points.append(pt)
  pub.publish(msg)

if __name__ == '__main__':
  rospy.init_node('waypoint_marker_publisher', anonymous=True)

  rospy.Subscriber("planner/waypoints/input", WaypointArray, input_waypoint_callback)
  rospy.Subscriber("planner/waypoints/optimized", WaypointArray, opt_waypoint_callback)
  rospy.spin()

