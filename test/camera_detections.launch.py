from threading import Event
import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_testing.actions
import launch_testing.markers
from launch.substitutions import PathJoinSubstitution
import pytest
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from threading import Thread

import pathlib
LAUNCH_PATH = pathlib.Path(__file__).parent.resolve()


@pytest.mark.launch_test
def generate_test_description():
    world = PathJoinSubstitution(
        [str(LAUNCH_PATH), 'camera_detections.world'])
    return LaunchDescription([
        ExecuteProcess(cmd=['gzserver', world, '--verbose']),
        launch_testing.actions.ReadyToTest()
    ])


class TestFixture(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_fusion_single_object(self):
        node = Listener(expected_no_of_msgs=1)
        thread = Thread(target=lambda: rclpy.spin(node))
        thread.start()
        msgs_received_flag = node.msgs_received_event.wait(timeout=5.0)
        assert msgs_received_flag, 'did not receive enough messages'

        msg = node.msgs[0]
        pose = msg.detections[0].results[0].pose.pose
        self.assertAlmostEqual(pose.position.x, 4.0)
        self.assertAlmostEqual(pose.position.y, 0.0)
        self.assertAlmostEqual(pose.position.z, -0.5)


class Listener(Node):

    def __init__(self, name='listener', expected_no_of_msgs=1):
        super().__init__(name)
        self.expected_no_of_msgs = expected_no_of_msgs
        self.msgs = []
        self.msgs_received_event = Event()
        self.sub_detections = self.create_subscription(
            Detection3DArray,
            'detections',
            self.msg_callback,
            2
        )

    def msg_callback(self, data):
        self.msgs.append(data)
        if len(self.msgs) >= self.expected_no_of_msgs:
            self.msgs_received_event.set()
