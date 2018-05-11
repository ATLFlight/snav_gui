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

from Tkinter import *
import rospy

import actionlib
import snav_msgs.msg

from std_msgs.msg import String
from snav_msgs.msg import WaypointWithConfigArray

class TopGui:

  def __init__(self):
    self.props_state = "NOT_SPINNING"
    self.master = Tk()
    self.debug_window_open = False
    self.tk_action_sv = StringVar()
    self.action = "NONE"
    self.tk_action_sv.set("Current action: {}".format(self.action))
    self.tk_status_sv = StringVar()
    self.tk_status_sv.set("")

    self.takeoff_client = actionlib.SimpleActionClient('takeoff', snav_msgs.msg.TakeoffAction)
    self.land_client = actionlib.SimpleActionClient('land', snav_msgs.msg.LandAction)
    self.execute_client = actionlib.SimpleActionClient('execute_mission', snav_msgs.msg.ExecuteMissionAction)
    self.go_to_client = actionlib.SimpleActionClient('go_to_waypoint', snav_msgs.msg.GoToWaypointAction)
    self.compute_traj_client = actionlib.SimpleActionClient('compute_traj', snav_msgs.msg.ComputeTrajAction)

    # Buttons
    self.b_takeoff_land = Button(self.master, text="Take Off", command=self.__takeoff, width=12)
    self.b_takeoff_land.grid(row=0, column=0)
    self.b_execute = Button(self.master, text="Execute Mission", command=self.__execute, width=12)
    self.b_execute.grid(row=0, column=1)
    self.b_recompute = Button(self.master, text="Recompute Traj", command=self.__recompute_traj, width=12)
    self.b_recompute.grid(row=0, column=2)
    self.b_clear = Button(self.master, text="Clear Waypoints", command=self.__clear, width=12)
    self.b_clear.grid(row=0, column=3)
    self.b_go_to = Button(self.master, text="Go To", command=self.__go_to, width=12)
    self.b_go_to.grid(row=1, column=0)
    self.b_abort = Button(self.master, text="Abort", command=self.__abort, width = 36)
    self.b_abort.grid(row=2, column=0, columnspan=4)

    # Entry
    self.waypoint = StringVar()
    self.e_wp = Entry(self.master, textvariable=self.waypoint, width=24)
    self.e_wp.grid(row=1, column=1, columnspan=2)
    self.e_wp.insert(0, "x, y, z, yaw")

    # Labels
    self.l_current_action = Label(self.master, textvariable=self.tk_action_sv, width=36)
    self.l_current_action.grid(row=3, column=0, columnspan=4)
    self.l_last_action_status = Label(self.master, textvariable=self.tk_status_sv, width=45)
    self.l_last_action_status.grid(row=4, column=0, columnspan=4)

    self.b_debug = Button(self.master, text="Debug", command=self.__create_debug_window, width=12)
    self.b_debug.grid(row=1, column=3)

    rospy.init_node("tk_test")
    rospy.Subscriber("props_state", String, self.__props_state_sub_cb)
    self.wp_pub = rospy.Publisher("input_waypoints", WaypointWithConfigArray, queue_size=10)


  def __props_state_sub_cb(self, data):
    self.props_state = data.data
    self.__update_takeoff_land_button()

  def __update_takeoff_land_button(self):
    if self.props_state == "NOT_SPINNING":
      self.b_takeoff_land.config(text="Take Off", command=self.__takeoff)
    else:
      self.b_takeoff_land.config(text="Land", command=self.__land)

  def wait_for_server(self):
    self.takeoff_client.wait_for_server()
    self.land_client.wait_for_server()
    self.execute_client.wait_for_server()
    self.go_to_client.wait_for_server()

  def __takeoff(self):
    if self.action == "NONE":
      goal = snav_msgs.msg.TakeoffGoal()
      self.takeoff_client.send_goal(goal, self.__done_cb, self.__takeoff_active_cb, self.__feedback_cb)

  def __takeoff_active_cb(self):
    self.__update_action("TAKEOFF")

  def __land(self):
    if self.action == "NONE":
      goal = snav_msgs.msg.LandGoal()
      self.land_client.send_goal(goal, self.__done_cb, self.__land_active_cb, self.__feedback_cb)

  def __land_active_cb(self):
    self.__update_action("LAND")

  def __abort(self):
    if self.action == "TAKEOFF":
      self.takeoff_client.cancel_goal()
    elif self.action == "LAND":
      self.land_client.cancel_goal()
    elif self.action == "EXECUTE":
      self.execute_client.cancel_goal()
    elif self.action == "GO TO WAYPOINT":
      self.go_to_client.cancel_goal()

  def __execute(self):
    if self.action == "NONE":
      goal = snav_msgs.msg.ExecuteMissionGoal()
      self.execute_client.send_goal(goal, self.__done_cb, self.__execute_active_cb, self.__feedback_cb)

  def __execute_active_cb(self):
    self.__update_action("EXECUTE")

  def __clear(self):
    self.wp_pub.publish(WaypointWithConfigArray())

  def __go_to(self):
    if self.action == "NONE":
      waypoint = self.waypoint.get().split(',')
      if len(waypoint) != 4:
        print("Waypoint input must be four comma-separated values")
        return
      goal = snav_msgs.msg.GoToWaypointGoal()
      goal.position.x = float(waypoint[0])
      goal.position.y = float(waypoint[1])
      goal.position.z = float(waypoint[2])
      goal.yaw = float(waypoint[3])
      self.go_to_client.send_goal(goal, self.__done_cb, self.__go_to_active_cb, self.__feedback_cb)

  def __go_to_active_cb(self):
    self.__update_action("GO TO WAYPOINT")

  def __recompute_traj(self):
    if self.action == "NONE":
      goal = snav_msgs.msg.ComputeTrajGoal()
      self.compute_traj_client.send_goal(goal, self.__done_cb, self.__recompute_traj_active_cb, self.__feedback_cb)

  def __recompute_traj_active_cb(self):
    self.__update_action("RECOMPUTE TRAJ")

  def __update_action(self, action):
    self.action = action
    self.tk_action_sv.set("Current action: {}".format(action))

  def __feedback_cb(self, feedback):
    if self.debug_window_open:
      self.debug_window_t.config(state=NORMAL)
      self.debug_window_t.delete(1.0, END)
      self.debug_window_t.insert(1.0, "Feedback of {}:\n{}\n".format(self.action, feedback))
      self.debug_window_t.config(state=DISABLED)

  def __done_cb(self, goal_status, result):
    self.tk_status_sv.set("{} finished with result: {}".format(self.action, self.__actionlib_status_to_string(goal_status)))
    if self.debug_window_open:
      self.debug_window_t.config(state=NORMAL)
      self.debug_window_t.delete(1.0, END)
      self.debug_window_t.insert(1.0, "Result of {}:\n{}\n".format(self.action, result))
      self.debug_window_t.config(state=DISABLED)
    self.__update_action("NONE")

  def __actionlib_status_to_string(self, status):
    if status == actionlib.GoalStatus.PENDING:
      return "PENDING"
    elif status == actionlib.GoalStatus.ACTIVE:
      return "ACTIVE"
    elif status == actionlib.GoalStatus.RECALLED:
      return "RECALLED"
    elif status == actionlib.GoalStatus.PREEMPTED:
      return "PREEMPTED"
    elif status == actionlib.GoalStatus.ABORTED:
      return "ABORTED"
    elif status == actionlib.GoalStatus.SUCCEEDED:
      return "SUCCEEDED"
    elif status == actionlib.GoalStatus.LOST:
      return "LOST"
    else:
      return "UNDEFINED"

  def __create_debug_window(self):
    if not self.debug_window_open:
      self.debug_window = Toplevel(self.master)
      self.debug_window.attributes('-topmost', True)
      self.debug_window.update()
      self.debug_window.protocol("WM_DELETE_WINDOW", self.__on_closing_debug_window)
      self.debug_window.wm_title("Action Debug")
      self.debug_window_s = Scrollbar(self.debug_window)
      self.debug_window_t = Text(self.debug_window, height=5, width=50)
      self.debug_window_s.pack(side=RIGHT, fill=Y)
      self.debug_window_t.pack(side=LEFT, fill=Y)
      self.debug_window_s.config(command=self.debug_window_t.yview)
      self.debug_window_t.config(yscrollcommand=self.debug_window_s.set)
      self.debug_window_t.config(state=DISABLED)
      self.debug_window_open = True

  def __on_closing_debug_window(self):
    self.debug_window_open = False
    self.debug_window.destroy()

  def run(self):
    self.master.attributes('-topmost', True)
    self.master.update()
    mainloop()

if __name__ == '__main__':
  gui = TopGui()
  gui.run()

