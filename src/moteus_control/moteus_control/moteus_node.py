# Copyright 2024 mjbots Robotic Systems, LLC.  info@mjbots.com
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from moteus_msgs.msg import PositionCommand


class MoteusNode(Node):

    def __init__(self):
        super().__init__('moteus_node')
        self.get_logger().info('Starting')
        self.subscription = self.create_subscription(
            PositionCommand,
            'position',
            self.position_command_callback,
            10)
        self.subscription  # prevent unused variable warning

    def position_command_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg}"')


def main(args=None):
    rclpy.init(args=args)

    node = MoteusNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
