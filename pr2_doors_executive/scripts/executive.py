#! /usr/bin/env python
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#         * Redistributions of source code must retain the above copyright
#             notice, this list of conditions and the following disclaimer.
#         * Redistributions in binary form must reproduce the above copyright
#             notice, this list of conditions and the following disclaimer in the
#             documentation and/or other materials provided with the distribution.
#         * Neither the name of the Willow Garage, Inc. nor the names of its
#             contributors may be used to endorse or promote products derived from
#             this software without specific prior written permission.
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
from pr2_controllers_msgs.msg import *
from std_msgs.msg import *

import threading

import door_functions

class SwitchControllersState(SimpleActionState):
    def __init__(self, stop_controllers, start_controllers):
        SimpleActionState.__init__(self,
                'switch_controllers',
                SwitchControllersAction,
                goal = SwitchControllersGoal(
                    stop_controllers = stop_controllers,
                    start_controllers = start_controllers))

# Door detector state
class DetectDoorState(State):
    def __init__(self, action_name):
        State.__init__(self, outcomes=['unlatched', 'closed', 'aborted'])
        self.ac = SimpleActionClient(action_name, DoorAction)
        if not self.ac.wait_for_server(rospy.Duration(30)):
            rospy.logerr('Door detector failed to connect to action server')
    def enter(self):
        if self.ac.send_goal_and_wait(DoorGoal(self.userdata.door), rospy.Duration(30), rospy.Duration(30)):
            result = self.ac.get_result() 
            self.userdata.door = result.door
            if self.userdata.door.latch_state == Door.UNLATCHED:
                return 'unlatched'
            else:
                return 'closed'
        return 'aborted'

def main():
    rospy.init_node('doors_executive')



    # construct state machine
    sm = StateMachine(['succeeded', 'aborted', 'preempted'])

    with sm:
        StateMachine.share_parent_userdata()
        StateMachine.add('INIT_CONTROLLERS',
                SwitchControllersState(
                    stop_controllers = ["r_arm_cartesian_tff_controller"],
                    start_controllers = ["r_arm_controller"]),
                { 'succeeded':'TUCK_ARMS' })
        StateMachine.add('TUCK_ARMS',
                SimpleActionState('tuck_arms', TuckArmsAction, TuckArmsGoal(True, True)),
                { 'succeeded': 'DETECT_DOOR',
                    'aborted': 'TUCK_ARMS'})
        
        StateMachine.add('DETECT_DOOR',
                DetectDoorState('detect_door'),
                { 'closed': 'DETECT_HANDLE',
                    'unlatched': 'aborted',
                    'aborted': 'DETECT_DOOR'})

        # Sequence for opening the door with the handle
        StateMachine.add('DETECT_HANDLE',
                SimpleActionState('detect_handle', DoorAction, goal_slots = ['door'], result_slots = ['door']),
                { 'succeeded': 'APPROACH_DOOR',
                    'aborted': 'DETECT_HANDLE'})

        def get_approach_goal(userdata, goal):
            target_pose = door_functions.get_robot_pose(userdata.door, -0.7)
            return MoveBaseGoal(target_pose)
        StateMachine.add('APPROACH_DOOR',
                SimpleActionState('pr2_move_base_local', MoveBaseAction, goal_cb = get_approach_goal),
                { 'succeeded':'GRASP_HANDLE',
                    'aborted':'APPROACH_DOOR'})

        StateMachine.add('UNTUCK_FOR_GRASP',
                SimpleActionState('tuck_arms', TuckArmsAction, TuckArmsGoal(True, False)),
                { 'succeeded': 'GRASP_HANDLE',
                    'aborted': 'aborted'})

        open_gripper_goal = Pr2GripperCommandGoal()
        open_gripper_goal.command.position = 0.07
        open_gripper_goal.command.max_effort = 99999
        StateMachine.add('OPEN_GRIPPER',
                SimpleActionState('r_gripper_controller/gripper_action',
                    Pr2GripperCommandAction,
                    goal = open_gripper_goal),
                {'succeeded':'GRASP_HANDLE'})

        StateMachine.add('GRASP_HANDLE',
                SimpleActionState('grasp_handle', DoorAction, goal_slots = ['door']),
                { 'succeeded':'TFF_START',
                    'aborted':'RECOVER_RELEASE_HANDLE'})

        StateMachine.add('RECOVER_RELEASE_HANDLE',
                SimpleActionState('release_handle',DoorAction,goal_slots = ['door']),
                { 'succeeded':'RECOVER_TUCK_ARMS',
                    'aborted':'RECOVER_RELEASE_HANDLE'})
        StateMachine.add('RECOVER_TUCK_ARMS', SimpleActionState('tuck_arms', TuckArmsAction, TuckArmsGoal(True, True)),
                { 'succeeded': 'APPROACH_DETECT_POSE',
                    'aborted': 'RECOVER_TUCK_ARMS'})

        def get_approach_detect_goal(userdata, goal):
            target_pose = door_functions.get_robot_pose(userdata.door, -1.5)
            return MoveBaseGoal(target_pose)
        StateMachine.add('APPROACH_DETECT_POSE',
                SimpleActionState('pr2_move_base_local', MoveBaseAction, goal_cb = get_approach_detect_goal),
                { 'succeeded':'DETECT_DOOR',
                    'aborted':'APPROACH_DETECT_POSE'})


        StateMachine.add('TFF_START',
                SwitchControllersState(
                    stop_controllers = ["r_arm_controller"],
                    start_controllers = ["r_arm_cartesian_tff_controller"]),
                { 'succeeded':'TURN_HANDLE' })

        StateMachine.add('TURN_HANDLE',
                SimpleActionState('unlatch_handle',DoorAction,goal_slots = ['door']),
                { 'succeeded':'OPEN_DOOR'})
        """
        StateMachine.add(
                label = 'TURN_HANDLE',
                state = SimpleActionState('unlatch_handle',DoorAction,goal_slots = ['door']),
                outcomes = { 'succeeded':'OPEN_DOOR'},
                resources = )
        """

        open_door = Concurrence(['succeeded','aborted','preempted'],default_outcome=['aborted'])
        with open_door:
            Concurrence.set_retrieve_ud_keys(['door'])
            Concurrence.add('MOVE_THROUGH',SimpleActionState('move_base_door',DoorAction, goal_slots = ['door']))
            Concurrence.add_slave('OPEN_DOOR',SimpleActionState('open_door',DoorAction, goal_slots = ['door']))
            Concurrence.add_outcome_map('succeeded',{'MOVE_THROUGH':'succeeded'})
            Concurrence.add_outcome_map('aborted',{'MOVE_THROUGH':'aborted'})
            Concurrence.add_outcome_map('preempted',{'MOVE_THROUGH':'preempted'})
        StateMachine.add('OPEN_DOOR',open_door,
                { 'succeeded':'TFF_STOP' })

        StateMachine.add('TFF_STOP',
                SwitchControllersState(
                    stop_controllers = ["r_arm_cartesian_tff_controller"],
                    start_controllers = ["r_arm_controller"]),
                { 'succeeded':'RELEASE_HANDLE' })

        StateMachine.add('RELEASE_HANDLE',
                SimpleActionState('release_handle',DoorAction,goal_slots = ['door']),
                { 'succeeded':'FINISH_TUCK_ARMS'})

        StateMachine.add('FINISH_TUCK_ARMS', SimpleActionState('tuck_arms', TuckArmsAction, TuckArmsGoal(True, True)),
                { 'succeeded': 'succeeded'})
        

        """
        # Sequence for pushing the door
        StateMachine.add('TOUCH_DOOR',
            SimpleActionState('touch_door', DoorAction, goal_slots = ['door']),
            {'succeeded':'PUSH_DOOR','aborted':'TOUCH_DOOR'})

        push_door = Concurrence(['succeeded','aborted','preempted'])
        push_door.add(
                ('PUSH_DOOR',SimpleActionState('push_door',DoorAction, goal_slots = ['door'])),
                ('MOVE_THROUGH',SimpleActionState('move_base_door',DoorAction, goal_slots = ['door'])))
        push_door.set_slave_states('PUSH_DOOR')
        push_door.add_outcome_map(
                ({'MOVE_THROUGH':'succeeded'},'succeeded'),
                ({'MOVE_THROUGH':'aborted'},'aborted'),
                ({'MOVE_THROUGH':'preempted'},'preempted'))
        StateMachine.add('PUSH_DOOR',push_door,{})
        """


    intro_server = IntrospectionServer('doorman',sm,'/DOORMAN')
    intro_server.start()

    action_server_acquire = ActionServerWrapper('move_through_door', DoorAction,
                                                wrapped_container = sm,
                                                goal_slots_map = {'door': 'door'},
                                                succeeded_outcomes = ['succeeded'],
                                                aborted_outcomes = ['aborted'],
                                                preempted_outcomes = ['preeempted'])
    action_server_acquire.run_server()

    rospy.spin()
    intro_server.stop()

if __name__ == '__main__':
        main()

