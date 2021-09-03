# HAROS Tutorials

This is a repository with example code to try out the various features of the [HAROS framework](http://github.com/git-afsantos/haros).

Hands-on videos following the instructions and exercises in this repository are available [on YouTube](https://youtube.com/playlist?list=PLrXxXaugT0cwVhjhlnxY6DU0_WYPLEmgq).

## What Is In The Box

This repository contains ROS packages that serve no particular purpose, other than to try out HAROS.

The `minimal_example` package is a minimal ROS package composed of two nodes with a ROS publisher, a ROS subscriber, a ROS service server, a ROS service client and a ROS parameter.

The `fictibot_*` packages make up a fictitious mobile robot system, called *Fictibot* (see [the docs](./docs)).
These packages provide software drivers for the mobile base, a random walker controller, a safety controller, a multiplexer to filter commands by priority, and message definitions.
Despite being a fictitious robot, the system can be executed normally, using `rosrun`, or `roslaunch` with the various provided launch files.

In addition to the ROS packages, this repository also contains example [project files](./projects) for HAROS, [scripts](./scripts) to help execute HAROS with the right arguments, and files to create a [Docker image](https://github.com/git-afsantos/haros_tutorials/tree/master/docker) with HAROS.

## Installing

> **Note:** If you want to try HAROS with Docker, you need only clone this repository and follow the [Docker instructions](https://github.com/git-afsantos/haros_tutorials/tree/master/docker).
>
> If you would rather follow a step by step installation, see the [HAROS instructions](https://github.com/git-afsantos/haros/blob/master/INSTALL.md) to install HAROS, and the remainder of this section for the ROS packages in this repository.
> The [*Make HAROS Easy* repository](https://github.com/git-afsantos/make-haros-easy) provides a number of convenience scripts that automate much of this process.

The first step is to ensure that you have a working [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
Depending on your setup, you might want to work with multiple workspaces, and create a new workspace for this tutorial.
Let us call it `haros_ws`.

Clone this repository into the source space of this workspace, by default `src`.

```bash
cd haros_ws/src
git clone https://github.com/git-afsantos/haros_tutorials.git
```

Simply build the packages with `catkin_make`, and you are ready to use the packages in a ROS environment.

```bash
cd ..
catkin_make
source devel/setup.bash
```

To use these packages with HAROS and Clang, an additional step is required.
You have to create a `compile_commands.json` file in the `build` directory of the workspace.
You can do it either with `catkin` or with `cmake`.

```bash
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_CXX_COMPILER=/usr/bin/clang++-6.0 src
```

Or

```bash
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_CXX_COMPILER=/usr/bin/clang++-6.0
```

Replace `src` in the first command with the source space of your workspace, if it is named differently.
Replace `/usr/bin/clang++-6.0` with the correct path to your `clang++` version.

> **Note:** it might be useful to turn this into a script and place it at the root of the workspace, so that you do not forget to update `compile_commands.json` when you build your ROS workspace.
 
```bash
#!/usr/bin/env bash
catkin_make --force-cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_CXX_COMPILER=/usr/bin/clang++-6.0
```

## Standalone Usage

To try out the ROS packages on their own, use any of the provided launch files.
They set up different use scenarios, sometimes showcasing some ROS features and quirks that might not be well known.

A normal use case, using the Fictibot nodes:

```bash
roslaunch fictibot_launch safe_random_walker.launch
```

Two Fictibot systems at the same time:

```bash
roslaunch fictibot_launch dual_bots.launch
```

Some quirks with `rosparam` that might not be obvious from the documentation (read the launch file's comments as well):

```bash
roslaunch minimal_example test_rosparam.launch
```

## Usage With HAROS

You can try out HAROS manually with any of the provided [project files](./projects).
Alternatively, feel free to use some pre-made [scripts](./scripts) that test out different features.

- [`scripts/minimal.sh`](./scripts/minimal.sh) runs model extraction on `minimal_example` and any available plugins.
- [`scripts/fictibot.sh`](./scripts/fictibot.sh) runs model extraction on various configurations of Fictibot and any available plugins.
- [`scripts/queries.sh`](./scripts/queries.sh) defines architectural queries for some configurations of Fictibot and executes [`haros_plugin_pyflwor`](https://github.com/git-afsantos/haros-plugin-pyflwor).
- [`scripts/hpl.sh`](./scripts/hpl.sh) defines [HPL properties](https://github.com/git-afsantos/hpl-specs) for the various Fictibot nodes and runs test generation with [`haros_plugin_pbt_gen`](https://github.com/git-afsantos/haros-plugin-pbt-gen).
- [`scripts/perf.sh`](./scripts/perf.sh) defines the ground truth of a Fictibot computation graph and evaluates the model extractor's accuracy with [`haros_plugin_model_ged`](https://github.com/git-afsantos/haros-plugin-model-ged).

## Bugs, Questions and Support

Please use the [issue tracker](https://github.com/git-afsantos/haros_tutorials/issues).

## Contributing

See [CONTRIBUTING](./CONTRIBUTING.md).

## Acknowledgment

Until March 2021, this work was financed by the ERDF – European Regional Development Fund through the Operational Programme for Competitiveness and Internationalisation - COMPETE 2020 Programme and by National Funds through the Portuguese funding agency, FCT - Fundação para a Ciência e a Tecnologia within project PTDC/CCI-INF/29583/2017 (POCI-01-0145-FEDER-029583).
