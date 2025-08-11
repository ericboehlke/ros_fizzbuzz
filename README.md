# FizzBuzz the ROS2 way

In this tutorial you will learn how to write the classic FizzBuzz programming game in ROS2 and Python 3.

The tutorial can be found on my website here: https://ericboehlke.com/ros2_exercises.html

This repository now contains two ROS 2 packages:

* `fizzbuzz_interfaces` – defines the `FizzBuzz` message type.
* `fizzbuzz` – Python nodes that publish numbers and compute fizz/buzz statistics.

Both packages use **ament_cmake** and are compatible with the Jazzy release. Build them with `colcon build`:

```bash
colcon build --packages-select fizzbuzz_interfaces fizzbuzz
```
