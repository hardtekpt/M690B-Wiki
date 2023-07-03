---
hide:
  - footer
---

# Usefull Packages

In order to facilitate testing and operation of the drone, two ROS packages are provided.

## mav_tools

The [mav_tools](https://github.com/hardtekpt/M690B-Wiki/tree/master/mavros_simple_interface/mav_tools) ROS package runs on top of MAVROS to provide a GUI interface that can run on the terminal and is able to display information like the battery state and basic control of the drone, like changing flight mode and arming/disarming the vehicle.

![mav_tools Example](../assets/mav_tools_example.png "mav_tools Example")

## test_comms_pixhawk

The [test_comms_pixhawk](https://github.com/hardtekpt/M690B-Wiki/tree/master/drone_control_stack/src/test_comms_pixhawk) package provides an easy way to test the communication from the ROS control stack to the pixhawk flight computer by logging telemetry data and arming and disarming the drone.