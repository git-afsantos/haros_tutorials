# Behaviour Specification and Property-based Testing

System behavior analysis aims to specify and verify diverse properties about the system's expected behavior.
It deals with the correctness of the software, regarding its requirements.

Verifying a robotic system's behavior is arguably the most challenging and interesting task in the whole analysis workflow from a software engineering perspective.
It is definitely the task that requires the most amount of work:

1. requirements must be analyzed for each system configuration;
2. specifying properties requires careful thought, to avoid logic mistakes;
3. verification techniques are harder to implement; and
4. the verification process takes longer than previous analyses.

HAROS encourages a *dependability case* approach, in which users specify properties about the system's behavior, possibly break them down into smaller sub-properties, and then gather evidence of such properties using a variety of techniques.
The properties are specified using the domain-specific [HPL language](https://github.com/git-afsantos/hpl-specs).
Then, coupling properties and extracted system models, different plugins handle the verification step.

For this tutorial, we will be using the plugin for [property-based testing](https://github.com/git-afsantos/haros-plugin-pbt-gen).
Given an architecture model and a testable HPL property, this plugin automatically generates tests that will try to break the property and find a counterexample.
It will also try to *shrink* the counterexample, so that it is presented in a manner that is as simple as possible to understand.

Quick links:

- [Exercise 1](#exercise-1)
- [Exercise 2](#exercise-2)

## Preparation

> Note: This tutorial assumes the use of the [Docker image](https://github.com/git-afsantos/haros_tutorials/tree/master/docker) provided in this repository.
> Other setups should work in a similar fashion, albeit with some adaptation to the console commands.

Property verification requires, first of all, an architecture model to be available.
Start with the same preparation steps given for the [Architecture section](https://github.com/git-afsantos/haros_tutorials/tree/master/exercises/sec3-architecture#preparation) before moving on.

## Introduction

Properties are specified relative to a concrete system or architecture.
They go under an `hpl:properties` section, under each of the `configurations` found in a project file, as we can see in the example [`hpl.yaml`](https://github.com/git-afsantos/haros_tutorials/blob/master/projects/hpl.yaml) project file.

```yaml
configurations:
    safe_random_walker:
        launch:
            - fictibot_launch/launch/safe_random_walker.launch
        hpl:
            properties:
                - "globally: no /laser {not data in [0 to 127]}"
                - "globally: no /vel {not linear in [MIN_LINEAR to MAX_LINEAR]}"
```

The same applies for specifications of individual `nodes`.

```yaml
nodes:
    fictibot_drivers/fictibot_driver:
        hpl:
            properties:
                - "globally: no /laser {not data in [0 to 127]}"
                - "globally: no /vel {not linear in [MIN_LINEAR to MAX_LINEAR]}"
```

These example properties are quite simple.
They have a `globally` *scope*, meaning that they should be true at all times.
The property itself is an *absence pattern* property, meaning that `no` single message matching the following pattern should be observed within the scope.

In this case, the given properties apply restrictions for the messages on the `/laser` and the `/vel` topics.
The first one, for example, says that `/laser` messages with a `data` field `not in` the range from `[0 to 127]` are forbidden.
The second property is similar, but forbids `linear` values of `/vel` messages outside the range from `[MIN_LINEAR to MAX_LINEAR]`, where these two values are constants defined in the respective [VelocityCommand](https://github.com/git-afsantos/haros_tutorials/blob/master/src/fictibot_msgs/msg/VelocityCommand.msg) message type.

To check if such properties hold, we will be using the [PBT plugin](https://github.com/git-afsantos/haros-plugin-pbt-gen) for HAROS, specifically the `v0.4` version.

> **Note:** the current version of this plugin has some limitations; not all properties are *testable*, in particular for the more complex scopes and patterns, such as `after _ until _: _ causes _ within _ ms`.

This plugin automatically generates a large number of tests for each property.
The limit is configurable and defaults to 100 tests per property.

For every test case, the plugin needs to launch the system under test, run the test, and take down the system, so it can start from scratch again with the following test example.
Naturally, this takes some time.
Expect testing to be much slower than previous types of analyses.
Furthermore, properties that are true will take longer to test because all tests must pass.
Properties that are false will break at some point and stop the tests earlier.

By default, the plugin only *generates* tests for the user to run at their leisure.
The generated tests can be found, by default, under `~/.haros/export/haros_plugin_pbt_gen`.
To run tests during the HAROS analysis (and to report test results in the HAROS visualizer), we need to add `user_data` in the project file, and set the `run_tests` setting to `true`.

```diff
 configurations:
     safe_random_walker:
         launch:
             - fictibot_launch/launch/safe_random_walker.launch
         hpl:
             properties:
                 - "globally: no /laser {not data in [0 to 127]}"
                 - "globally: no /vel {not linear in [MIN_LINEAR to MAX_LINEAR]}"
+        user_data:
+            haros_plugin_pbt_gen:
+                run_tests: true
```

To make things easier to test, and to grasp the properties, we will be focusing on testing individual nodes in the following exercises.

## Exercise 1.A

> **Difficulty:** Easy  
> **Plugin:** `haros_plugin_pbt_gen`  
> **Package:** `fictibot_safety_controller`  
> **File:** `src/safety_controller.cpp`

### Problem

Look at the documentation for the [safety controller node](https://github.com/git-afsantos/haros_tutorials/tree/master/docs#fictibot_safety_controller).

For the `/laser` subscriber, it is stated that it:

> publishes a velocity command of zero (in both components) for laser values of 64 or lower, or publishes an emergency stop request for values of 32 or lower.

Let us construct a property for the first part of this behaviour.

1. The property should clearly hold at all times, so the scope is `globally`.
2. A message is published in *response* to a received message. This duality can be expressed with the `causes-within` pattern - one message *leads to* the publication of another within a certain time window.
3. The trigger message must be a `/laser` message with `data` values of 64 or lower.
4. The response message is a `/cmd_vel` message with a `linear` value of 0 and an `angular` value of 0.
5. The exact response time is not stated, but it is implied that it is immediate. A safe limit of 200 milliseconds should suffice in all cases.

Putting together all the pieces, we get:

```
globally: /laser {data <= 64} causes /cmd_vel {linear = 0.0 and angular = 0.0} within 200 ms
```

Now, we need to express the second part with another property.
Specifically,

> *(in response to a `/laser` message, the safety controller node)* publishes an emergency stop request for values of 32 or lower.

Adapting the previous example, we get the following property.

```
globally: /laser {data <= 32} causes /cmd_stop within 200 ms
```

However, this property is not correctly implemented.

Open the `fictibot.yaml` project file.  
Your task is to add the `fictibot_safety_controller/fictibot_safety_controller` node to a `nodes` section in this file, and annotate it with this second property.
Set `run_tests` to `true`, run HAROS, and then analyse the reported issue.

From the given counterexample, try to understand what went wrong.
Open the `safety_controller.cpp` file, find the bug and fix it.

### Solution

The first step is rather straightforward.
We must add the node specification to the project file, as shown in the [diff file](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec4-behaviour/ex1.diff).

```diff
             - name: One Publisher Per Topic
               details: "Found {n} topics with multiple publishers - {entities}"
               query: "topics[len(self.publishers) > 1]"
+nodes:
+    fictibot_safety_controller/fictibot_safety_controller:
+        hpl:
+            properties:
+                - "globally: /laser {data <= 32} causes /cmd_stop within 200 ms"
+        user_data:
+            haros_plugin_pbt_gen:
+                run_tests: true
 analysis:
```

Then, we run the analysis again, which should complete after a few minutes.
We can see that adding this new node also added a new model to the visualizer.

![Analysis Screen 1](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec4-behaviour/screen1.png)

Clicking on `Issues` should take us to the counterexample found via testing.

![Analysis Screen 2](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec4-behaviour/screen2.png)

The issue tells us which property was violated, although, in this case, we only have one.
Then, it shows us the counterexample that turned the property false.

Counterexamples reported by this plugin are **traces of messages**.
In this case, the trace is composed of a single entry.
At the 1,131 millisecond mark, after launching the node, the testing plugin sent to the safety controller a `/laser` message with `data = 0`.

This message is called a *witness* because it is relevant for the property's structure.
Recall that the property is a binary relation, where a specific `/laser` message `causes` a `/cmd_stop` message to be published, at most after 200 milliseconds.
The message that the testing plugin sent satisfies the pattern, `data = 0 <= 32`.
However, the safety controller node did not publish the expected response within the time limit.

Locate the `safety_controller.cpp` file and navigate to the laser message callback.

```cpp
void SafetyController::laser_callback(const std_msgs::Int8::ConstPtr& msg)
{
    if (!safe_) { return; }

    // laser data in [0, 127] (cm)
    if (msg->data <= 64)
    {
        slow_down();
    }
}
```

It is no wonder that a `/cmd_stop` message was not sent.
The special case for `data <= 32` is not implemented at all!
The laser callback only checks for values below 64 to send a slow down signal.
It must also check for values below 32 to send the emergency stop.

The solution is to add an additional `if` statement, to check for values below 32, as shown in the [diff file](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec4-behaviour/ex1.diff).
We can use the `SafetyController::emergency_stop()` implemented in the same file to publish the stop message.

```diff
     // laser data in [0, 127] (cm)
-    if (msg->data <= 64)
+    if (msg->data <= 32)
+    {
+        emergency_stop();
+    }
+    else if (msg->data <= 64)
     {
         slow_down();
     }
```

After implementing this change, do not forget to go back to the root of the workspace and run `./catkin.sh` to compile the new code.
Compilation is not necessary for static analyses, but now we are running tests; we need an up-to-date binary executable.

## Exercise 1.B

> **Difficulty:** Easy  
> **Plugin:** `haros_plugin_pbt_gen`  
> **Package:** `fictibot_safety_controller`  
> **File:** `src/safety_controller.cpp`

### Problem

Re-run the analysis, after implementing the changes required in the previous exercise.
It should take a longer time to finish now, as it becomes harder for the testing plugin to falsify the property.
You will notice, though, that even after applying the changes it still reports a property violation, but now with a different counterexample.

![Analysis Screen 3](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec4-behaviour/screen3.png)



> **Note:** even with this solution the tests keep failing, because the code has a mechanism to prevent message spam `if (!safe_) {return;}`, and this causes a *second* `/laser` message to go by without a response.

```
[Example #58]: FAIL
>> @3,037ms sent spam on /laser
data: 0
>> @3,238ms sent witness on /laser
(same as above)
```

## Exercise 2

> **Difficulty:** Hard  
> **Plugin:** `haros_plugin_pbt_gen`  
> **Package:** `fictibot_random_controller`  
> **File:** `src/random_controller.cpp`

### Problem

The following property appears to be true, at a first glance, but is, in fact, false.

`globally: /bumper {left or center or right} causes /cmd_vel within 200 ms`

The counterexample is the following:

```
>> @687ms sent witness on /bumper
    left: True
    center: False
    right: False
>> @687ms sent spam on /bumper
    left: False
    center: False
    right: False
```

And the buggy section of the code is this:

```cpp
if (msg->left) {
    bumper_left_pressed_ = true;
} else {
    bumper_left_pressed_ = false;
}
```

It is a classic of internal state being processed only every *t* seconds, and sending two rapid-fire messages that cancel each other.

### Solution

In this case, the solution might be to do nothing, as shown in the [diff file](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec4-behaviour/ex2.diff), and assume that the property is wrong.

Alternatively, one can assign `false` to the internal flags only after publishing the reaction message; or one could publish the reaction message immediately in the callback.
