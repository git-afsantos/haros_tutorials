# Internal Code Quality

The term *internal code quality* is quite vague, as the quality of a piece of code can be determined by many factors.
In general, though, it refers to writing code in such a manner that it is easy to read, understand, maintain and reuse.
It is easier to read linear and modular code, than it is to read [*spaghetti code*](https://en.wikipedia.org/wiki/Spaghetti_code), for example, especially when working with a team.
It is also easier to design and write tests for a small function with few `if` statements than it is for a large function with many possible cases.

Internal code quality is commonly evaluated based on a number of metrics, such as lines of code and cyclomatic complexity, or by checking compliance with coding standards, such as the [ROS C++ Style Guide](https://wiki.ros.org/CppStyleGuide).

We tackle code quality first, in this tutorial, as it is mostly a ROS-agnostic and application-independent problem.
While refactoring the code is laborious, the analysis is fast, and the results immediate.
This step unveils general programming mistakes and bad practices that, once fixed, can potentially reduce the time spent on debugging the application-specific issues detected with other analyses, as the code becomes more maintainable.

Quick links:

- [Exercise 1](#exercise-1)
- [Exercise 2](#exercise-2)
- [Exercise 3](#exercise-3)

## Preparation

To illustrate the variety and number of issues that are related to internal code quality, we can start with a very small example.

The minimalistic [`quality.yaml` project file](https://github.com/git-afsantos/haros_tutorials/blob/master/projects/quality.yaml) defines only a list of all Fictibot packages that we want to analyse.

Execute HAROS with this project file as its input.

```bash
cd src/haros_tutorials
haros full -s 0.0.0.0:8080 -p projects/quality.yaml
```

> **Note:** the `-s 0.0.0.0:8080` option is only necessary if running HAROS from the [Docker image](https://github.com/git-afsantos/haros_tutorials/tree/master/docker) provided in this repository. It makes the visualization server accessible on `http://localhost:8080` for web browsers on the host machine.

Next, open your web browser on `http://localhost:8080` and check the analysis results.

![Analysis Screen 1](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec2-code-quality/screen1.png)

You should see a fairly large number of reported issues, and, upon inspection, you may also notice that many of these issues are related to coding style.

> **Note:** Coding style is ultimately a matter of preference.
> Considering that it takes a non-trivial amount of time to fix manually, we will skip it in this tutorial.
> In a realistic scenario, we recommend automatic tools to keep formatting consistent, such as [ROSCpp Code Format](https://github.com/PickNikRobotics/roscpp_code_format).

These issues can be filtered out in the visualizer, but we can skip the analysis and reporting of these issues entirely with two methods.

**Method 1:** adding filters to the project file.
Add the following settings to the bottom of the `quality.yaml` project file.

```diff
     - fictibot_msgs
     - fictibot_launch
+ analysis:
+     ignore:
+         tags:
+             - formatting
+             - copyright
```

**Method 2:** blacklist uninteresting plugins or whitelist only a subset.
For the following exercises, we need only whitelist `haros_plugin_cppcheck` and `haros_plugin_lizard`.

Run the modified HAROS command, reload the web browser, and notice how the number of reported issues diminishes greatly.

```bash
haros full -s 0.0.0.0:8080 -w haros_plugin_cppcheck haros_plugin_lizard -p projects/quality.yaml
```

With this setup, we are now ready to move into the actual exercises.

## Exercise 1

> **Difficulty:** Easy  
> **Plugin:** `haros_plugin_cppcheck`  
> **Package:** `fictibot_random_controller`  
> **File:** `src/random_controller.cpp`

### Problem 

There is an uninitialized variable in the Fictibot Random Controller package.

![Analysis Screen 2](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec2-code-quality/screen2.png)

Uninitialized variables are an issue because, depending on the compiler, the values they produce on read operations could be [unpredictable](https://stackoverflow.com/a/30172680) and end up changing program logic.
They may even change every time the faulty function is called.

Your task is to locate the uninitialized variable and assign it an initial value.

### Solution

Go to the file and line of code pointed by HAROS, and simply write down the initial value of `false`, as shown in the [diff file](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec2-code-quality/ex1.diff).

```diff
 void RandomController::spin()
 {
-    bool obstacle;
+    bool obstacle = false;
```

Upon running the analysis again and refreshing the web browser, you should no longer see this issue.

## Exercise 2

> **Difficulty:** Easy  
> **Plugin:** `haros_plugin_cppcheck`  
> **Package:** `fictibot_drivers`  
> **File:** `src/motor_manager.cpp`

### Problem 

Two variables are not declared at their smallest possible scope.

![Analysis Screen 3](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec2-code-quality/screen3.png)

This issue does not represent a threat, but declaring variables at their smallest possible scope helps make the code more readable.
Readers are not left guessing where the variable is going to be used.

Your task is to locate these variables and move them down to the appropriate scope level.

### Solution

Go to the file and line of code pointed by HAROS, and simply move down the variable declarations of `delta_v` and `limit` to the inside of the `if` statement, as shown in the [diff file](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec2-code-quality/ex2.diff).

```diff
-    double delta_v, limit;
     if (!stopped)
     {
-        delta_v = command_.linear - velocity_.linear;
-        limit = LINEAR_ACCEL * delta_t_;
+        double delta_v = command_.linear - velocity_.linear;
+        double limit = LINEAR_ACCEL * delta_t_;
```

Upon running the analysis again and refreshing the web browser, you should no longer see this issue.

## Exercise 3

> **Difficulty:** Intermediate  
> **Plugin:** `haros_plugin_lizard`  
> **Package:** `fictibot_drivers`  
> **File:** `src/motor_manager.cpp`

### Problem 

The `void spin()` function is too long and has a high cyclomatic complexity.

![Analysis Screen 4](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec2-code-quality/screen4.png)

Complex functions have too many decision points (`if` statements, `for` statements, etc.).
They are harder for readers to understand and they are more difficult to test fully, because you have to consider all possible scenarios.

Long functions are just a lot of code to scroll through.
It is likely that such functions are performing more than one action/resposibility, and it is often better to split such functions into smaller chunks, especially if code duplication is involved.

Your task is to locate the faulty function and refactor it, so that it becomes both shorter and less complex.

### Solution

**Step 1:** Go to the file and line of code pointed by HAROS and study the `spin()` function.

The function has two sections that are nearly identical, save for changing different variables.
One is an `if` statement regarding linear velocity (`command_.linear`) and another is an `if` statement regarding angular velocity (`command_.angular`).
These sections are self-contained - meaning that one does not interfere or depend on the other - and can be transferred to their own functions.
Splitting `spin()` into three functions, as shown in the [diff file](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec2-code-quality/ex3.diff) clears both issues.

**Step 2:** Open the header file `fictibot_drivers/motor_manager.h` and add two new functions.

```diff
     void velocity_callback(const fictibot_msgs::VelocityCommand::ConstPtr& msg);
+
+    void update_linear_vel();
+    void update_angular_vel();
```

**Step 3:** Implement the new functions in the `cpp` file by moving the respective `if` statements from `spin()`, and add calls to these new functions in `spin()`.

```cpp
void MotorManager::spin()
{
     ...
     if (!stopped)
     {
       update_linear_vel();
       update_angular_vel();
     }
```
