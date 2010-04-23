#! /usr/bin/env python
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Wim Meeussen


import roslib; roslib.load_manifest('pr2_doors_executive')
import rospy

from actionlib import *
from actionlib.msg import *
from smach import *
from executive_python_msgs.msg import *
from door_msgs.msg import *
from move_base_msgs.msg import *
from pr2_common_action_msgs.msg import *
from std_msgs.msg import *



def doorResultCb(userdata, result_status, result):
    userdata.door = result.door

def doorGoalCb(userdata, goal):
    return DoorGoal(userdata.door)


def main():
    rospy.init_node('doors_executive')

    # initial door
    prior_door = Door()
    prior_door.frame_p1.x = 1.0
    prior_door.frame_p1.y = -0.5
    prior_door.frame_p2.x = 1.0
    prior_door.frame_p2.y = 0.5
    prior_door.door_p1.x = 1.0
    prior_door.door_p1.y = -0.5
    prior_door.door_p2.x = 1.0
    prior_door.door_p2.y = 0.5
    prior_door.travel_dir.x = 1.0
    prior_door.travel_dir.y = 0.0
    prior_door.travel_dir.z = 0.0
    prior_door.rot_dir = Door.ROT_DIR_COUNTERCLOCKWISE
    prior_door.hinge = Door.HINGE_P2
    prior_door.header.frame_id = "base_footprint"

    # Door detector state
    class DetectDoorState(State):
        def __init__(self, action_name):
            State.__init__(self, outcomes=['open', 'closed', 'aborted'])
            self.ac = SimpleActionClient(action_name, DoorAction)
            if not self.ac.wait_for_server(rospy.Duration(30)):
                rospy.logerr('Door detector failed to connect to action server')
        def enter(self):
            if self.ac.send_goal_and_wait(DoorGoal(self.userdata.door), rospy.Duration(30), rospy.Duration(30)):
                self.userdata.door = self.ac.get_result().door
                if self.userdata.door.latch_state == Door.UNLATCHED:
                    return 'open'
                else:
                    return 'closed'
            else:
                return 'aborted'

    # construct state machine
    sm = StateMachine(['succeeded', 'aborted', 'preempted'])
    sm.local_userdata.door = prior_door

    sm.add(('TUCK_ARMS', SimpleActionState('tuck_arms', TuckArmsAction, TuckArmsGoal(False, True, True)),
            {'succeeded': 'DETECT_DOOR', 'aborted': 'TUCK_ARMS', 'preempted': 'preempted'}))
    
    sm.add(('DETECT_DOOR', DetectDoorState('detect_door'),
            {'closed': 'DETECT_HANDLE', 'open': 'aborted', 'aborted': 'DETECT_DOOR'}))

    sm.add(('DETECT_HANDLE', SimpleActionState('detect_handle', DoorAction, goal_cb = doorGoalCb, result_cb = doorResultCb),
            {'succeeded': 'succeeded', 'aborted': 'DETECT_HANDLE', 'preempted': 'preempted'}))


    sm.set_initial_state(['TUCK_ARMS'])

    sm.enter()

    print sm.local_userdata.door


if __name__ == "__main__":
    main()
