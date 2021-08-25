# Behaviour Specification and Property-based Testing

System behavior analysis aims to specify and verify diverse properties about the system's expected behavior.
It deals with the correctness of the software, regarding its requirements.

Verifying the correctness, or functional safety, of a system is highly application-specific.
It depends on what the system does, and on the concrete mission or application.
Regardless, it is one of the most sought-after analyses overall.

HAROS encourages a dependability case approach, in which users specify properties about the system's behavior, possibly break them down into smaller sub-properties, and then gather evidence of such properties using a variety of techniques.
The properties are specified using the domain-specific [HPL language](https://github.com/git-afsantos/hpl-specs).
Then, coupling properties and extracted system models, different plug-ins handle the verification step.

For this tutorial, we will be using the plugin for [property-based testing](https://github.com/git-afsantos/haros-plugin-pbt-gen).
Given an architecture model and a testable HPL property, this plugin automatically generates tests that will try to break the property and find a counterexample.
It will also try to *shrink* the counterexample, so that it is presented in a manner that is a simple as possible to understand.

Quick links:

- [Exercise 1](#exercise-1)
- [Exercise 2](#exercise-2)

## Preparation

TBA

With this setup, we are now ready to move into the actual exercises.

## Exercise 1

> **Difficulty:** Easy  
> **Plugin:** `haros_plugin_pbt_gen`  
> **Package:** `fictibot_safety_controller`  
> **File:** `src/safety_controller.cpp`

### Problem

The following property is documented for the Fictibot safety controller, but is not correctly implemented.

`globally: /laser {data <= 32} causes /cmd_stop within 200 ms`

The problem lies in the laser callback, that only checks for values below 64 to send a slow down signal.
It is missing the additional check for values below 32 to send the emergency stop.

```cpp
if (msg->data <= 64) {
    slow_down();
}
```

### Solution

The solution is to add an additional `if` statement, to check for values below 32, as shown in the [diff file](https://github.com/git-afsantos/haros_tutorials/blob/master/exercises/sec4-behaviour/ex1.diff).

```cpp
if (msg->data <= 32) {
    emergency_stop();
} else if (msg->data <= 64) {
    slow_down();
}
```

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
