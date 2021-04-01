#! /usr/bin/env python

import rospy
import os
import subprocess


class DummyProcess:
    def __init__(self):
        pass

    def terminate(self):
        pass


class RobustGazeboLaunch:
    def __init__(self):
        rospy.init_node('robust_gazebo_launch')

        self.received_model_states = False
        self.launch_cmd = 'roslaunch ' + rospy.get_param('~launch_cmd', default='gazebo_ros empty_world.launch')
        self.gazebo_process = DummyProcess()

        self.start_gazebo()

    def start_gazebo(self):
        self.stop_gazebo()
        self.gazebo_process = subprocess.Popen(self.launch_cmd.split())

    def stop_gazebo(self):
        self.gazebo_process.terminate()
        os.system('killall gzserver')
        os.system('killall gzclient')

    def shutdown_handler(self):
        self.stop_gazebo()


if __name__ == '__main__':
    try:
        node_instance = RobustGazeboLaunch()
        rospy.on_shutdown(node_instance.shutdown_handler)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
