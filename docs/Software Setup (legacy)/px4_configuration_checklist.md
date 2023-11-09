---
hide:
  - footer
---

# PX4 Configuration Checklist

After assembling the vehicle, the [**PX4** ](https://docs.px4.io/main/en/) autopilot running on the flight computer must be configured. Here is a general checklist of what needs to be done before the first flight. All of these configurations can be done using the [**QGroundControl**](http://qgroundcontrol.com/) application. It should be noted that this checklist is based on the PX4 documentation and more detailed instructions are available [there](https://docs.px4.io/main/en/config/).

1.  Sensor orientation on vehicle;
2.  Compass calibration;
3.  Gyroscope calibration;
4.  Accelerometer calibration;
5.  Radio setup (button mapping and calibration);
6.  Flight mode configuration (map radio button for manual/offboard mode change and safety hold/kill switch button);
7.  Battery (configure battery parameters for charge estimation);
8.  Configure failsafes;
9.  Actuators (configure geometry of motors and verify correct direction of spinning);

After this process the vehicle should be ready for manual flight and it is now a good idea to check if the PID controllers' gains need to be adjusted.