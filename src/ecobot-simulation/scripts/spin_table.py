#!/usr/bin/env python3.8
from absl import app
import yaml
import rospy
from gazebo_msgs.srv import SetLinkState
from gazebo_msgs.msg import LinkState
from rospy_tutorials.msg import Floats
from std_msgs.msg import String


rospy.init_node('spinning_table')
with open(rospy.get_param('~config_path'), "r") as f:
    config = yaml.safe_load(f)

config = config['table_params']
time_delta = config['table_time_delay']


class Table(object):
    def __init__(
        self,
        in_arm_status_topic: str,
        in_table_topic: str
    ):
        self.arm_msg = None
        rospy.Subscriber(in_table_topic, Floats, self.callback)
        rospy.Subscriber(
            in_arm_status_topic,
            String,
            self.arm_msg_callback
        )

    def stop_table(self):
        state = LinkState()
        state.link_name = 'Rot_table::link_1'
        state.reference_frame = 'Rot_table::link_0'
        state.twist.angular.z = 0
        set_state = rospy.ServiceProxy('/gazebo/set_link_state', SetLinkState)
        set_state(state)

    def arm_msg_callback(self, msg):
        self.arm_msg = msg.data

    def callback(self, msg):
        target_time = msg.data[1]
        if target_time:
            cur_time = rospy.Time.now().to_sec()
            while cur_time < target_time - time_delta:
                cur_time = rospy.Time.now().to_sec()
            while not self.arm_msg:
                self.stop_table()
            self.arm_msg = None
        # TODO: launch table again

    def main(self):
        rospy.spin()


def main_action(_argv):
    act = Table(
        in_arm_status_topic=rospy.get_param('~in_arm_status_topic'),
        in_table_topic=rospy.get_param('~in_table_topic')
    )
    act.main()


if __name__ == '__main__':
    app.run(main_action)
