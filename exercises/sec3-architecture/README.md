# Computation Graph Extraction and Queries

System architecture analysis aims to detect orchestration problems - i.e., problems related to how nodes are set up and connected to each other - without executing the system.
For example, it can be used to detect mismatching message types, where one node publishes messages of type *X* on topic *t*, while another node subscribes to messages of type *Y* on the same topic.
It can also be used to detect missing remappings, bad namespace management, or more straightforward issues, such as the use of global ROS names.

Architectural analysis requires the model extraction capabilities of HAROS that, while mostly automatic, are unable to handle all cases in general.
To remedy this, HAROS provides mechanisms for users to specify [extraction hints](https://github.com/git-afsantos/haros/blob/master/docs/USAGE.md#defining-custom-applications), so that a sound and complete model is achieved.
However, for the basic exercises with Fictibot, these should not be required.

Besides manual inspection of the graphical models, we will be using the HAROS [plug-in for PyFLWOR](https://github.com/git-afsantos/haros-plugin-pyflwor), that enables user-defined queries that match and report problematic patterns in the model.

Quick links:

- [Exercise 1](#exercise-1)
- [Exercise 2](#exercise-2)

## Preparation

> **Note:** This tutorial assumes the use of the [Docker image](https://github.com/git-afsantos/haros_tutorials/tree/master/docker) provided in this repository.
> Other setups should work in a similar fashion, albeit with some adaptation to the console commands.

To enable the automatic extraction of architectural models from the source code, HAROS must parse the source code, as if compiling it.
In a system such as Fictibot, written in C++, this requires a few steps prior to the actual extraction, due to how C++ is parsed and compiled.

### Step 1 - Compilation Database File

Generate a `compile_commands.json` file in the `build` directory of your workspace.

> **Note:** executing the [`catkin.sh` script](https://github.com/git-afsantos/haros_tutorials/blob/master/docker/catkin.sh) in the provided Docker container will do this automatically for you.

This file can be generated either with `cmake` or with `catkin_make`.
For example commands and more details see the instruction [at the root](https://github.com/git-afsantos/haros_tutorials#installing) of this repository or the [FAQ](https://github.com/git-afsantos/haros/blob/master/docs/FAQ.md#the-models-only-show-nodes) about HAROS.

> **Note:** This file must be generated again if significant changes to the source code occur (e.g., adding new files or changing included headers).

> **Note:** This step requires the `clang++` compiler.

### Step 2 - Configure HAROS

Ensure that HAROS knows the paths to your workspace and to the Clang libraries.

> **Note:** This is automatically done for you in the provided Docker image.

After running HAROS for the first time, the HAROS home directory should be available.
By default, it sits at `~/.haros`.
In this directory, you should find a `configs.yaml` file that you need to edit.

See the [HAROS instructions](https://github.com/git-afsantos/haros/blob/master/docs/USAGE.md#settings-file) for the meaning and expected value of each field, or see the [settings file](https://github.com/git-afsantos/haros_tutorials/blob/master/docker/configs.yaml) we use in the Docker image for an example.

### Step 3 - Define ROS Systems

Add some configurations to your HAROS project file.

> **Note:** The Docker image provides project files that do this for you, such as [`fictibot.yaml`](https://github.com/git-afsantos/haros_tutorials/blob/master/projects/fictibot.yaml).

To do this, you should add a `configurations` section to the project file where you define systems with lists of launch files.
See the [HAROS instructions](https://github.com/git-afsantos/haros/blob/master/docs/USAGE.md#defining-custom-applications) for more details.

### Step 4 - Run HAROS

To enable model extraction you should add the `-n` option to your commands.
Try it with the `fictibot.yaml` project as an example.

```bash
cd src/haros_tutorials
haros full -s 0.0.0.0:8080 -n -p projects/fictibot.yaml
```

> **Note:** extracting models takes roughly as much time as it takes to compile the code.
> HAROS uses some caching mechanisms, but a considerable slow down is to be expected when running for the first time.

Once the analysis completes, open your web browser at `http://localhost:8080` and navigate to the *Models* tab at the top.
You should see a number of models available for visualization, such as the `dual_bots` configuration.

![Extracted Model 1](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec3-architecture/screen1.png)

With this setup, we are now ready to move into the actual exercises.  
From this point onward, you may also use the [`tutorial_script.sh` script](https://github.com/git-afsantos/haros_tutorials/blob/master/scripts/tutorial_script.sh) found in this repository, instead of typing the full HAROS command every time.
It uses the `fictibot.yaml` project file as its input.

```bash
cd src/haros_tutorials
./tutorial_script.sh
```

## Introduction

Some issues can be detected simply by being familiar with the system's design and looking at the extracted models.
Take the `dual_bots` model, for example.

Looking at the documentation for [Fictibot's architecture](https://github.com/git-afsantos/haros_tutorials/tree/master/docs#ros-architecture), specifically the [Deployment section](https://github.com/git-afsantos/haros_tutorials/tree/master/docs#deployment), we can see that this model is supposed to launch two independent systems; one under the `/one` namespace and another under the `/two` namespace.
But in the computation graph shown above, we can clearly see that one topic is shared between both systems, the `/cmd_vel` topic.
This is not supposed to happen.

We can spot another issue, also with the `/cmd_vel` topic, in the `safe_random_walker` configuration.
The random controller node publishes on this topic, but there are no subscribers.
From the documentation, we would expect this controller to be publishing on the *normal priority* input of the multiplexer instead.

![Extracted Model 2](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec3-architecture/screen2.png)

In the following exercises, we will detect and repair these types of problems.

## Exercise 1

> **Difficulty:** Easy  
> **Plugin:** None  
> **Package:** `fictibot_random_controller`  
> **File:** `src/random_controller.cpp`

### Problem

HAROS automatically reports that, in the Fictibot random controller package, a topic publisher uses the global name `/cmd_vel` in the `advertise()` function.

![Analysis Screen 1](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec3-architecture/screen3.png)

Global names are discouraged, because they make it more troublesome to instantiate multiple nodes of the same type, or organise components under namespaces.
And we have seen exactly this type of problem in the diagrams above, with this exact node and topic.

Your task is to change the global name into a relative name, and then inspect the launch files in `fictibot_launch` to see if any `remap` tag is affected.

### Solution

Locate the faulty call to `advertise()` in the file and line pointed by HAROS.
Then, remove the forward slash in the topic name, as shown in the [diff file](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec3-architecture/ex1.diff).

```diff
-    command_publisher_ = n.advertise<fictibot_msgs::VelocityCommand>("/cmd_vel", 1);
+    command_publisher_ = n.advertise<fictibot_msgs::VelocityCommand>("cmd_vel", 1);
```

This suffices to resolve the issue, but we have to check that, by fixing this issue, we are not introducing new bugs in other models.
Check if any launch file relies on the global name we had prior, for example with `grep`,

```bash
grep "cmd_vel" src/fictibot_launch/launch/*.launch
```

Which produces:

```
src/fictibot_launch/launch/multiplexer.launch:
    <remap from="cmd_vel" to="high_cmd_vel" />
src/fictibot_launch/launch/multiplexer.launch:
    <remap from="cmd_stop" to="high_cmd_vel" />
src/fictibot_launch/launch/safe_random_walker.launch:
    <remap from="cmd_vel" to="normal_cmd_vel" />
src/fictibot_launch/launch/safe_random_walker.launch:
    <remap from="cmd_vel" to="high_cmd_vel" />
src/fictibot_launch/launch/type_check.launch:
    <remap from="cmd_stop" to="cmd_vel" />
```

There are no `remap from="/cmd_vel"`, meaning that, in principle, this change should be safe to make.

Upon running the analysis again and refreshing the web browser, the two previously incorrect models, `dual_bots` and `safe_random_walker`, should change to their correct form.

```bash
haros full -s 0.0.0.0:8080 -n -p projects/fictibot.yaml
```

![Extracted Model 3](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec3-architecture/screen4.png)

## Exercise 2

> **Difficulty:** Intermediate  
> **Plugin:** `haros_plugin_pyflwor`  
> **Configuration:** `multiplex`  
> **Package:** `fictibot_launch`  
> **File:** `launch/multiplex.launch`

### Problem

The random controller node in the `multiplex` configuration is missing a topic remapping from `cmd_vel` to `normal_cmd_vel`.
Without it, there are two publishers on the `cmd_vel` topic subscribed by the base, the random controller and the multiplexer.

The safety controller node in the same configuration has a copy-and-paste error where both `cmd_vel` and `cmd_stop` are redirected to `high_cmd_vel`.

### Solution

Writing a query to detect multiple publishers on a single topic detects both issues.

```yaml
user_data:
  haros_plugin_pyflwor:
    - name: One Publisher Per Topic
      details: "Found {n} topics with multiple publishers - {entities}"
      query: "topics[len(self.publishers) > 1]"
```

The fix for the former is to add the missing remapping to the launch file.
The fix for the latter is to correct the remapped topic name.
See the [diff file](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec3-architecture/ex2.diff).

```diff
   <node name="ficticontrol" pkg="fictibot_random_controller" type="fictibot_random_controller">
+    <remap from="cmd_vel" to="normal_cmd_vel" />
   </node>
 
   <node name="fictisafe" pkg="fictibot_safety_controller" type="fictibot_safety_controller">
     <remap from="cmd_vel" to="high_cmd_vel" />
-    <remap from="cmd_stop" to="high_cmd_vel" />
+    <remap from="cmd_stop" to="high_cmd_stop" />
   </node>
```

Upon running the analysis again and refreshing the web browser, you should no longer see this issue.
