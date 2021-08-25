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

TBA

With this setup, we are now ready to move into the actual exercises.

## Exercise 1

> **Difficulty:** Easy  
> **Plugin:** `haros_plugin_pyflwor`  
> **Package:** `fictibot_random_controller`  
> **File:** `src/random_controller.cpp`

### Problem

The `cmd_vel` topic publisher uses a global name in the `advertise()` function.
Global names are discouraged, because they make it more troublesome to instantiate multiple nodes of the same type, or organise components under namespaces.

This issue makes two of the extracted models incorrect.
The affected graphs are `dual_bots` and `safe_random_walker`.

### Solution

It suffices to remove the forward slash in the topic name, as shown in the [diff file](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec3-architecture/ex1.diff).

```diff
-    command_publisher_ = n.advertise<fictibot_msgs::VelocityCommand>("/cmd_vel", 1);
+    command_publisher_ = n.advertise<fictibot_msgs::VelocityCommand>("cmd_vel", 1);
```

Upon running the analysis again and refreshing the web browser, you should no longer see this issue.

## Exercise 2

> **Difficulty:** Intermediate  
> **Plugin:** `haros_plugin_pyflwor`  
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
