# Basic HAROS Tutorial

This directory provides a series of basic exercises to try out [HAROS](https://github.com/git-afsantos/haros) using the ROS packages provided in this repository as a target application for analysis.
Hands-on videos following the instructions and exercises are available [on YouTube](https://youtube.com/playlist?list=PLrXxXaugT0cwVhjhlnxY6DU0_WYPLEmgq).

The exercises cover various types of analyses with progressing difficulty, from internal code quality to architecture analysis and property-based testing.
If applicable, solutions for the exercises are also provided.
We recommend that the user should get familiar with Fictibot's architecture (see [the docs](https://github.com/git-afsantos/haros_tutorials/tree/master/docs)), if not with the source code itself, in order to better understand the issues HAROS reports, as well as the suggested solutions.

The exercises assume an installation via Docker, using the files provided [in this repository](https://github.com/git-afsantos/haros_tutorials/tree/master/docker), to ensure a common ground.
Other types of HAROS installations should work similarly, but some commands may have to be adapted (e.g., replacing `haros` with `python haros-runner.py` if running from source).

> **Note:** for convenience, instead of typing the haros commands every time, you may simply run the [`tutorial_script.sh` script](https://github.com/git-afsantos/haros_tutorials/blob/master/scripts/tutorial_script.sh).
> This script uses the [`fictibot.yaml` project](https://github.com/git-afsantos/haros_tutorials/blob/master/projects/fictibot.yaml) as its input.
> You may edit this file as you progress through the exercises.

## Internal Code Quality

Exercises about making code easier to read and maintain, as well as catching general programming bugs, such as uninitialised variables.

See the [Internal Code Quality directory](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec2-code-quality).

## Computation Graph Extraction and Queries

Exercises about extracting a Computation Graph model from the source code and then using queries to detect problematic patterns, such as having multiple publishers on the same topic.

See the [Architecture directory](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec3-architecture).

## Behaviour Specification and Property-based Testing

Exercises about specifying behavioural properties, such as what types of messages a node should and should not publish, and then using automated tests to check the properties.

See the [Behaviour directory](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec4-behaviour).

## Developing New Plug-ins

TBA
