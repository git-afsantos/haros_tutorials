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

TBA

## Exercise 2

TBA

## Exercise 3

TBA
