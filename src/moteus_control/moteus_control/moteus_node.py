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

'''A ROS2 Node that allows commanding and monitoring one or more moteus
controllers.

The ROS2 argument args should be populated with command line arguments
like those passed to tview or moteus_tool in order to specify a
transport and any transport-specific arguments.  `moteus_tool --help`
will show what those are in a given installation.

The ROS2 argument 'ids' contains a list of moteus IDs to control.

NOTE: Commands are only sent to moteus when they are received via ROS.
That means that if you have a watchdog timeout configured with
`servo.default_timeout_s` (the default is 0.1), then you must send ROS
commands at least that often or the servo will enter the position
timeout state.  To avoid this, either send ROS commands more often
than the configured watchdog, or change or disable the watchdog.

'''

# TODO:
#
# * Get shutdown to be error free.
# * Verify it works with a pi3hat.
# * have some timeouts to better handle missing servos

import argparse
import asyncio
import functools
import moteus
import rclpy
from rclpy.node import Node
import std_msgs
import std_msgs.msg
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


def _extract_position_command(msg):
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


def _extract_empty(msg):
    return {}


def _extract_single_position(msg):
    return {'position': msg.data}


# The moteus python library is single threaded and relies on asyncio.
# rclpy is multi-threaded and calls things from arbitrary threads.  To
# make those work together, we create a separate thread to run all
# asyncio things, and associate a single asyncio event loop with that.
# Then every ROS callback's only job is to enqueue an appropriate
# function to be invoked in that thread using
# 'asyncio.run_coroutine_threadsafe'.
#
# We assume it is safe to invoke rclpy functions from the asyncio
# thread.
class MoteusNode(Node):
    def __init__(self, loop):
        super().__init__('moteus_node')

        self.initialize_future = None
        self.loop = loop

        # Which IDs to communicate with.
        self.declare_parameter('ids', [1])

        # Any transport specific arguments, specified as if presented
        # on the command-line.  For example ["--force-transport",
        # "fdcanusb"] would require a fdcanusb be used as a transport.
        self.declare_parameter('args', [''])

        # Command are sent as they are received from ROS.  The status
        # of the servos is queried at a periodic rate, defined by this
        # parameter.
        self.declare_parameter('query_period', 0.02)

        # If we send a "stop" command to each servo upon starting.
        # This is on by default in order to clear any faults that may
        # have been present from prior operation.
        self.declare_parameter('initial_stop', True)

        self.initialize_future = asyncio.run_coroutine_threadsafe(
            self.async_initialize(), self.loop)

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

        initial_stop = self.get_parameter('initial_stop').value

        id_list = self.get_parameter('ids').value

        for id in id_list:
            self.controllers[id] = moteus.Controller(
                id=id,
                transport=self.transport,
                query_resolution=query_resolution)

            if initial_stop:
                await self.controllers[id].set_stop()

            CALLBACKS = [
                (PositionCommand, 'cmd_position',
                 moteus.Controller.set_position,
                 _extract_position_command),
                (std_msgs.msg.Empty, 'cmd_stop',
                 moteus.Controller.set_stop,
                 _extract_empty),
                (std_msgs.msg.Empty, 'cmd_brake',
                 moteus.Controller.set_brake,
                 _extract_empty),
                (std_msgs.msg.Float32, 'cmd_set_output_exact',
                 moteus.Controller.set_output_exact,
                 _extract_single_position),
                (std_msgs.msg.Float32, 'cmd_set_output_nearest',
                 moteus.Controller.set_output_nearest,
                 _extract_single_position),
                (std_msgs.msg.Empty, 'cmd_recapture_position_velocity',
                 moteus.Controller.set_recapture_position_velocity,
                 _extract_empty),
            ]

            for message, name, moteus_command, extract in CALLBACKS:
                self.my_subscriptions.append(
                    self.create_subscription(
                        message,
                        f'id_{id}/{name}',
                        functools.partial(
                            self.command_callback,
                            id, moteus_command, extract),
                        10))

            self.my_publishers[id] = self.create_publisher(
                ControllerState, f'id_{id}/state', 10)

        query_period = self.get_parameter('query_period').value
        if query_period > 0.0:
            self.timer = self.create_timer(
                query_period, self.query_callback)

        self.get_logger().info(
            f'Started with ids {list(self.controllers.keys())}')

    def command_callback(self, id, moteus_command, extract, msg):
        future = asyncio.run_coroutine_threadsafe(
            self.async_command_callback(id, moteus_command, extract, msg),
            self.loop)
        future.result()

    async def async_command_callback(self, id, moteus_command, extract, msg):
        controller = self.controllers[id]
        await moteus_command(controller, **extract(msg))

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

        if node.initialize_future:
            # So that we'll see any errors that happen during
            # initialization.
            node.initialize_future.result()

        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        node.destroy_node()


if __name__ == '__main__':
    main()
