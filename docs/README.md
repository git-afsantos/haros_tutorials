# The Fictibot System

This repository contains ROS packages that make up a fictitious mobile robot system, called Fictibot.
The main purpose of these packages is to use them as a target application to learn how to use [HAROS](https://github.com/git-afsantos/haros).

## Concept and Hardware

Fictibot represents a somewhat typical mobile robot base with a few incorporated sensors.
The main inspiration behind this robot was [TurtleBot2](https://www.turtlebot.com/turtlebot2/) (see the picture below), so you can imagine Fictibot as being similar to this platform, in terms of its *fictitious* hardware.

![Picture of TurtleBot2](https://www.turtlebot.com/assets/images/turtlebot2e.png)

### Sensors

In terms of sensors, Fictibot has:

- three **bumper** sensors on its front (left, center and right);
- one **laser** scanner (straight line in front of the robot);
- two **wheel drop** sensors (left wheel and right wheel);
- an **emergency stop** button.

Each of the **bumper** sensors can be in one of two states, either *pressed* or *released*.

The **laser** scanner reports distance values to the nearest obstacle in front of the robot.
The values are reported in centimeters, in the range from 0 to 128 (exclusive).
If there are no obstacles in front of the robot, it should always report 127, the maximum distance.

Each of the **wheel drop** sensors can be in one of two states, either *dropped* (no contact with the floor) or *raised* (in contact with the floor).

Pressing the **emergency stop** button should stop the robot *immediately*.
This effect should last for a configurable safety period (5 seconds by default).

### Actuators

The main actuators of Fictibot are its two wheels, to provide movement.
Movement is specified in terms of a target (desired) **linear velocity** (m/s) and **angular velocity** (rad/s).

A **linear velocity** value of 0 means a full stop, in terms of *translation*.
Positive values mean forward movement, while negative values mean backward movement.
The minimum value is -1 m/s. The maximum value is +1 m/s.

Linear acceleration is not instantaneous.
The robot accelerates, at most, 0.5 m/s².

An **angular velocity** value of 0 means a full stop, in terms of *rotation*.
Positive values mean rotation to the left, while negative values mean rotation to the right.
The minimum value is -PI rad/s. The maximum value is +PI rad/s.

Angular acceleration is not instantaneous.
The robot accelerates, at most, `PI/2` rad/s².

## ROS Architecture

The full Fictibot system is composed of four different nodes, connected via topics as per the attached computation graph ([PDF](https://github.com/git-afsantos/haros_tutorials/blob/master/docs/fictibot.pdf), [PNG](https://raw.githubusercontent.com/git-afsantos/haros_tutorials/master/docs/fictibot.png)).
The subsections below provide details into each of the different types of node found in the diagram.


### `fictibot_driver`

This node controls the sensors and actuators of Fictibot.
Given that Fictibot is not an actual robot, the node simulates hardware by providing seemingly random values for its sensor readings.
The full source code can be found in the [`fictibot_drivers` package](https://github.com/git-afsantos/haros_tutorials/tree/master/src/fictibot_drivers).

**Subscribers**

- `cmd_stop` (`std_msgs/Empty`): receiving a message on this topic means that the emergency stop button has been pressed.
- `cmd_vel` ([`fictibot_msgs/VelocityCommand`](https://github.com/git-afsantos/haros_tutorials/blob/master/src/fictibot_msgs/msg/VelocityCommand.msg)): receiving a message on this topic sets the current target velocity (with linear and angular velocity components); it does *not* change the current velocity of the robot, only its goal.

**Publishers**

- `vel` ([`fictibot_msgs/VelocityCommand`](https://github.com/git-afsantos/haros_tutorials/blob/master/src/fictibot_msgs/msg/VelocityCommand.msg)): publishes the current velocity of the robot (with linear and angular velocity components).
- `bumper` ([`fictibot_msgs/BumperEvent`](https://github.com/git-afsantos/haros_tutorials/blob/master/src/fictibot_msgs/msg/BumperEvent.msg)): publishes the latest readings of the state of the bumper sensors; the message contains boolean fields for the `left`, `center` and `right` sensors, where a value of `true` means *pressed* and a value of `false` means *released*.
- `laser`(`std_msgs/Int8`): publishes the latest readings of the state of the laser scanner sensor; the message contains a single `data` value holding the distance to the nearest obstacle in centimeters, from 0 to 127 (inclusive).
- `wheel` ([`fictibot_msgs/WheelDropEvent`](https://github.com/git-afsantos/haros_tutorials/blob/master/src/fictibot_msgs/msg/WheelDropEvent.msg)): publishes the latest readings of the state of the wheel drop sensors; the message contains boolean fields for the `left` and `right` sensors, where a value of `true` means *dropped* and a value of `false` means *raised*.

**Parameters**

- `stop_time` (`double`): the time, in seconds, during which the robot should remain stopped, after pressing the emergency stop button; defaults to 5 seconds.


### `fictibot_random_controller`

This node issues new velocity commands to the robot, chosen at random and changing periodically.
If the robot runs into an obstacle, a new command is chosen in response.
The full source code can be found in the [`fictibot_random_controller` package](https://github.com/git-afsantos/haros_tutorials/tree/master/src/fictibot_random_controller).

**Subscribers**

- `bumper` ([`fictibot_msgs/BumperEvent`](https://github.com/git-afsantos/haros_tutorials/blob/master/src/fictibot_msgs/msg/BumperEvent.msg)): stores the bumper state to process at the next `spin` iteration; values of `true` in any field trigger a command change.
- `laser`(`std_msgs/Int8`): stores the laser state to process at the next `spin` iteration; values of 50 or lower trigger a command change.
- `wheel` ([`fictibot_msgs/WheelDropEvent`](https://github.com/git-afsantos/haros_tutorials/blob/master/src/fictibot_msgs/msg/WheelDropEvent.msg)): stores the wheel drop state to process at the next `spin` iteration; values of `true` in any field trigger a command change.

**Publishers**

- `cmd_vel` ([`fictibot_msgs/VelocityCommand`](https://github.com/git-afsantos/haros_tutorials/blob/master/src/fictibot_msgs/msg/VelocityCommand.msg)): publishes a new target velocity (with linear and angular velocity components).

**Parameters**

- `change_time` (`double`): controls the frequency (in seconds) at which a new velocity command is chosen; defaults to 5 seconds.


### `fictibot_safety_controller`

This reactive node issues velocity and emergency stop commands in response to the sensors of Fictibot.
The messages are published only once every *t* seconds, with *t* being a configurable value.
The full source code can be found in the [`fictibot_safety_controller` package](https://github.com/git-afsantos/haros_tutorials/tree/master/src/fictibot_safety_controller).

**Subscribers**

- `bumper` ([`fictibot_msgs/BumperEvent`](https://github.com/git-afsantos/haros_tutorials/blob/master/src/fictibot_msgs/msg/BumperEvent.msg)): publishes an emergency stop request in response to values of `true` in any field.
- `laser`(`std_msgs/Int8`): publishes a velocity command of zero (in both components) for laser values of 64 or lower, or publishes an emergency stop request for values of 32 or lower.
- `wheel` ([`fictibot_msgs/WheelDropEvent`](https://github.com/git-afsantos/haros_tutorials/blob/master/src/fictibot_msgs/msg/WheelDropEvent.msg)): publishes an emergency stop request in response to values of `true` in any field.

**Publishers**

- `cmd_vel` ([`fictibot_msgs/VelocityCommand`](https://github.com/git-afsantos/haros_tutorials/blob/master/src/fictibot_msgs/msg/VelocityCommand.msg)): publishes a new target velocity (with linear and angular velocity components) in response to certain readings of the laser scanner sensor.
- `cmd_stop` (`std_msgs/Empty`): publishes an emergency stop request in response to certain readings of bumper, laser or wheel drop sensors.

**Parameters**

- `reaction_time` (`double`): the time (in seconds) during which the node avoids sending further (repeating) messages; defaults to 1 second.

### `fictibot_multiplex`

This node is a multiplexer for the different sources of velocity and emergency stop commands of Fictibot.
It provides three channels with different priority levels, namely a *low* priority channel, a *normal* priority channel and a *high* priority channel.
Messages of higher priority are forwarded and set a temporary lock period, during which messages of lower priorities are dropped.
Initially, messages of all priorities are allowed.
After a period of inactivity (defaults to 1 second), the priority locks are reset to the initial state.
The full source code can be found in the [`fictibot_multiplex` package](https://github.com/git-afsantos/haros_tutorials/tree/master/src/fictibot_multiplex).

**Subscribers**

- `low_cmd_vel` ([`fictibot_msgs/VelocityCommand`](https://github.com/git-afsantos/haros_tutorials/blob/master/src/fictibot_msgs/msg/VelocityCommand.msg)): input channel for *low* priority velocity messages.
- `low_cmd_stop` (`std_msgs/Empty`): input channel for *low* priority emergency stop messages.
- `normal_cmd_vel` ([`fictibot_msgs/VelocityCommand`](https://github.com/git-afsantos/haros_tutorials/blob/master/src/fictibot_msgs/msg/VelocityCommand.msg)): input channel for *normal* priority velocity messages.
- `normal_cmd_stop` (`std_msgs/Empty`): input channel for *normal* priority emergency stop messages.
- `high_cmd_vel` ([`fictibot_msgs/VelocityCommand`](https://github.com/git-afsantos/haros_tutorials/blob/master/src/fictibot_msgs/msg/VelocityCommand.msg)): input channel for *high* priority velocity messages.
- `high_cmd_stop` (`std_msgs/Empty`): input channel for *high* priority emergency stop messages.

**Publishers**

- `cmd_vel` ([`fictibot_msgs/VelocityCommand`](https://github.com/git-afsantos/haros_tutorials/blob/master/src/fictibot_msgs/msg/VelocityCommand.msg)): forwards all received velocity command messages that pass through the priority filter.
- `cmd_stop` (`std_msgs/Empty`): forwards all received emergency stop messages that pass through the priority filter.
- `state` (`std_msgs/Int8`): publishes changes to the priority filter state of the multiplexer; negative values mean that all priorities are enabled (*low*, *normal* and *high*); a value of 0 means that only *normal* priority and *high* priority messages are allowed; positive values mean that only *high* priority messages are allowed.
