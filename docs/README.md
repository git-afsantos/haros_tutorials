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
- two **wheel drop** sensors (left wheel and right wheel).

Each of the **bumper** sensors can be in one of two states, either *pressed* or *released*.

The **laser** scanner reports distance values to the nearest obstacle in front of the robot.
The values are reported in centimeters, in the range from 0 to 128 (exclusive).
If there are no obstacles in front of the robot, it should always report 127, the maximum distance.

Each of the **wheel drop** sensors can be in one of two states, either *dropped* (no contact with the floor) or *raised* (in contact with the floor).

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

The second actuator of Fictibot is an **emergency stop** button.
Pressing this button should stop all movement *immediately* and its effects should last for a configurable safety period (5 seconds by default).

## ROS Architecture

TODO

see [BumperEvent.msg](https://github.com/git-afsantos/haros_tutorials/blob/master/src/fictibot_msgs/msg/BumperEvent.msg)
