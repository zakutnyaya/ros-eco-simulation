#!/usr/bin/env python3.8
import math
import yaml
from absl import app
import actionlib
import rospy
from std_msgs.msg import Header, Bool, String
from rospy_tutorials.msg import Floats
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import (
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryAction,
    JointTolerance
)


rospy.init_node('robot_grabbing_object')
with open(rospy.get_param('~config_path'), "r") as f:
    config = yaml.safe_load(f)

initial_position = config['robot_params']['initial_position']
robot_joints_names = config['robot_params']['robot_joints_names']
grab_trajectory_dict = config['robot_params']['grab_trajectory_dict']
time_delta_dict = config['robot_params']['time_delta_dict']
arm_time_delay = config['robot_params']['arm_time_delay']
table_velocity = config['table_params']['table_velocity']
table_big_radius = config['table_params']['table_big_radius']


class GetCoordinatesMove:
    def __init__(
        self,
        out_arm_status_topic: str,
        trajectory_client_topic: str,
        in_detection_topic: str
    ):
        self.arm_status_pub = rospy.Publisher(
            out_arm_status_topic,
            String,
            queue_size=1
        )
        self.arm_client = actionlib.SimpleActionClient(
            trajectory_client_topic,
            FollowJointTrajectoryAction
        )
        self.arm_client.wait_for_server()

        self.send_trajectory(initial_position, start_time=0)

        rospy.Subscriber(
            in_detection_topic,
            Floats,
            self.coordinates_callback,
            queue_size=1
        )

    def send_trajectory(
        self,
        time_coord_dict: dict,
        start_time: float
    ) -> None:
        joints_str = JointTrajectory()
        joints_str.header = Header()
        if start_time:
            joints_str.header.stamp = start_time
        else:
            joints_str.header.stamp = rospy.Time.now()

        joints_str.joint_names = robot_joints_names
        for key in time_coord_dict.keys():
            point = JointTrajectoryPoint()
            point.positions = time_coord_dict[key]
            point.time_from_start = rospy.Duration(key)
            joints_str.points.append(point)

        arm_goal_pos = FollowJointTrajectoryGoal()
        arm_goal_pos.trajectory = joints_str
        tolerance_fingers = JointTolerance()
        tolerance = JointTolerance()
        tolerance.position = 0
        tolerance_fingers.position = 1
        arm_goal_pos.path_tolerance = [
            tolerance,
            tolerance,
            tolerance_fingers,
            tolerance_fingers,
            tolerance_fingers,
            tolerance_fingers
        ]
        self.arm_client.send_goal(arm_goal_pos)
        self.arm_client.wait_for_result()

    def create_trajectory_dict(
        self,
        y_coordinate: float,
        move_time: float
    ) -> dict:
        trajectory_dict = {}
        time = move_time
        for joint, time_delta in time_delta_dict.items():
            time += time_delta
            trajectory = grab_trajectory_dict[joint]
            trajectory[0] *= y_coordinate
            trajectory_dict[time] = trajectory
        return trajectory_dict

    def coordinates_callback(self, coord_msg: Floats) -> None:
        target_y, target_time = coord_msg.data
        n = abs(table_velocity) * 180 / math.pi
        secs_per_meter = 1 / (math.pi * table_big_radius * n / 180)
        move_time = arm_time_delay + secs_per_meter * target_y
        trajectory_dict = self.create_trajectory_dict(target_y, move_time)
        self.send_trajectory(trajectory_dict, rospy.Time.from_sec(target_time))
        self.arm_status_pub.publish('done')

    def main(self):
        rospy.spin()


def main_action(_argv):
    action = GetCoordinatesMove(
        out_arm_status_topic=rospy.get_param('~out_arm_status_topic'),
        trajectory_client_topic=rospy.get_param('~trajectory_client_topic'),
        in_detection_topic=rospy.get_param('~in_detection_topic')
    )
    action.main()


if __name__ == '__main__':
    app.run(main_action)
