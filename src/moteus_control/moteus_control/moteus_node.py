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
# * Get shutdown to be error free.
# * Verify it works with multiple devices.
# * Verify it works with a pi3hat.
# * Make sample launch file.
# * Mark dependency on 'moteus' pypi package.
# * Make the initial "stop" command configurable.
# * Maybe make query resolution configurable?

import argparse
import asyncio
import functools
import moteus
import rclpy
from rclpy.node import Node
import threading

from moteus_msgs.msg import PositionCommand, ControllerState

QUERY_NAMES = [
    (moteus.Register.MODE, 'mode'),
    (moteus.Register.POSITION, 'position'),
    (moteus.Register.VELOCITY, 'velocity'),
    (moteus.Register.TORQUE, 'torque'),
    (moteus.Register.MOTOR_TEMPERATURE, 'motor_temperature'),
    (moteus.Register.TRAJECTORY_COMPLETE, 'trajectory_complete'),
    (moteus.Register.HOME_STATE, 'home_state'),
    (moteus.Register.VOLTAGE, 'voltage'),
    (moteus.Register.TEMPERATURE, 'temperature'),
    (moteus.Register.FAULT, 'fault_code'),
]


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
        self.loop = loop

        self.declare_parameter('ids', [1])
        self.declare_parameter('args', [''])
        self.declare_parameter('query_period', 0.02)

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

        query_resolution = moteus.QueryResolution()
        query_resolution.motor_temperature = moteus.F32
        query_resolution.trajectory_complete = moteus.INT8
        query_resolution.home_state = moteus.INT8


        self.my_publishers = {}
        self.my_subscriptions = []
        for id in self.get_parameter('ids').value:
            self.controllers[id] = moteus.Controller(
                transport=self.transport,
                query_resolution=query_resolution)

            await self.controllers[id].set_stop()

            self.my_subscriptions.append(
                self.create_subscription(
                    PositionCommand,
                    f'id_{id}/position',
                    functools.partial(self.position_command_callback, id),
                    10))
            self.my_publishers[id] = self.create_publisher(
                ControllerState, f'id_{id}/state', 10)

        query_period = self.get_parameter('query_period').value
        if query_period > 0.0:
            self.timer = self.create_timer(query_period, self.query_callback)

        self.get_logger().info(
            f'Started with ids {list(self.controllers.keys())}')

    def position_command_callback(self, id, msg):
        future = asyncio.run_coroutine_threadsafe(
            self.async_position_command_callback(id, msg), self.loop)
        future.result()

    async def async_position_command_callback(self, id, msg):
        controller = self.controllers[id]
        await controller.set_position(**_extract(msg))

    def query_callback(self):
        future = asyncio.run_coroutine_threadsafe(
            self.async_query_callback(), self.loop)
        future.result()

    async def async_query_callback(self):
        commands = [
            x.make_query()
            for x in self.controllers.values()
        ]
        results = await self.transport.cycle(commands)
        for result in results:
            publisher = self.my_publishers.get(result.id, None)
            if publisher is None:
                continue

            msg = ControllerState()
            for key, name in QUERY_NAMES:
                if key in result.values:
                    setattr(msg, name, result.values[key])

            publisher.publish(msg)


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
