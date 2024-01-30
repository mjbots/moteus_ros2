# ROS2 Package for moteus

This contains a ROS2 node for communication with mjbots moteus controllers.

- https://mjbots.com
- https://github.com/mjbots/moteus

License: Apache 2.0

# Getting started

## Installation

1. Clone this repository into the `src` folder of your ROS2 workspace.

```
cd src
git clone https://github.com/mjbots/moteus_ros2
```


2. Navigate to the top level of the ROS2 workspace folder and run:

```
colcon build
```

3. Source the workspace:

```
source ./install/setup.bash
```

4. Run the example launch file:

```
ros2 launch moteus_control example_launch.yaml
```

## Monitoring and control.

The following commands assume you are using the parameters from the
default launch file.  You can change the namespace or the IDs
configured according to your application.  The following are examples
of commands that can be sent.

1. Show the current status of servo ID 1.

```
ros2 topic echo /moteus/id_1/state
```

2. Request the motor stop, in order to clear any faults.

```
ros2 topic pub /moteus/id_1/cmd_stop std_msgs/msg/Empty "{}"
```


3. Send a position mode command.

The following requests a move to position 0.5, repeating the command
at 50Hz.  Each field which should be sent, should be a single element
list with the desired value present.

```
ros2 topic pub -r 50 /moteus/id_1/cmd_position moteus_msgs/msg/PositionCommand "{position: [0.5], accel_limit: [5]}"
```

Note that the ROS2 node only sends commands to moteus when it receives
them from ROS.  Thus you need to either:

a. Send ROS commands faster than the configured watchdog timeout
period (0.1s by default)

b. Change `servo.default_timeout_s` to `nan` if you want it disabled

c. Temporarily override the watchdog timeout on a per-command basis
like the following:

```
ros2 topic pub /moteus/id_1/cmd_position moteus_msgs/msg/PositionCommand "{position: [0.5], accel_limit: [5], watchog_timeout:[.NAN]}"
```

4. Send a command that requires no arguments.  The possible options
are:

```
/moteus/id_1/cmd_stop
/moteus/id_1/cmd_brake
/moteus/id_1/cmd_recapture_position_velocity
```

These can all be sent with the following:

```
ros2 topic pub -1 /moteus/id_1/cmd_brake std_msgs/msg/Empty "{}"
```

5. Send a command that requires a single floating point argument.  The possible options are:

```
/moteus/id_1/cmd_set_output_exact
/moteus/id_1/cmd_set_output_nearest
```

These can be sent with something of the form:

```
ros2 topic pub -1 /moteus/id_1/cmd_set_output_exact std_msgs/msg/Float32 "{data: 0.5}"
```

# Reference

The semantics of all commands are identical to that found in the primary moteus reference documentation: https://github.com/mjbots/moteus/blob/main/docs/reference.md
