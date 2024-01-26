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

# TODO:
#
# * Read status for all devices periodically if configured to do so.
# * Send commands periodically if configured to do so (maybe make that
#   be the default?)
# * Get shutdown to be error free.
# * Verify it works with multiple devices.
# * Verify it works with a pi3hat.
# * Maybe always request trajectory_complete flag?

import argparse
import asyncio
import functools
import moteus
import rclpy
from rclpy.node import Node
import threading

from moteus_msgs.msg import PositionCommand


def _extract(msg):
    result = {}
    for key in msg.__slots__:
        if not key.startswith('_'):
            continue
        if key == '_check_fields':
            continue
        maybe_value = getattr(msg, key)
        if len(maybe_value) >= 1:
            result[key[1:]] = maybe_value[0]
    return result


class MoteusNode(Node):

    def __init__(self, loop):
        super().__init__('moteus_node')
        self.get_logger().info('Starting')

        self.loop = loop

        self.declare_parameter('ids', [1])
        self.declare_parameter('args', [''])

        asyncio.run_coroutine_threadsafe(self.async_initialize(), self.loop)

    async def async_initialize(self):
        parser = argparse.ArgumentParser()
        moteus.make_transport_args(parser)
        ros_args = self.get_parameter('args').value
        if len(ros_args) == 1 and ros_args[0] == '':
            ros_args = []
        args = parser.parse_args(ros_args)
        self.transport = moteus.get_singleton_transport(args)
        self.controllers = {}

        self.my_subscriptions = []
        for id in self.get_parameter('ids').value:
            self.controllers[id] = moteus.Controller(
                transport=self.transport)

            await self.controllers[id].set_stop()

            self.my_subscriptions.append(
                self.create_subscription(
                    PositionCommand,
                    f'id_{id}/position',
                    functools.partial(self.position_command_callback, id),
                    10))

    def position_command_callback(self, id, msg):
        future = asyncio.run_coroutine_threadsafe(
            self.async_position_command_callback(id, msg), self.loop)
        future.result()

    async def async_position_command_callback(self, id, msg):
        controller = self.controllers[id]
        self.get_logger().info(f'Sending cmd {id} {msg}')

        await controller.set_position(**_extract(msg))
        print('command sent2')
        self.get_logger().info(f'Command sent')


def main(args=None):
    rclpy.init(args=args)
    loop = asyncio.new_event_loop()

    node = MoteusNode(loop)

    try:
        ros_thread = threading.Thread(target = rclpy.spin, args = (node,))
        ros_thread.start()
        loop.run_forever()
    finally:
        loop.close()

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()


if __name__ == '__main__':
    main()
