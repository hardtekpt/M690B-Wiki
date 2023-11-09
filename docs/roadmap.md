---
hide:
  - footer
  - navigation
  - toc
---

# Roadmap

- [x] Verify drone instrumentation: 
    * [x] Install OS - Jetpack 5
    * [x] Install ROS and setup workspace
    * [x] Setup the VRPN ROS package
    * [x] Setup the MAVROS package
    * [x] Verify communication between MAVROS and the Pixhawk
    * [x] Install the ROS control stack
    * [x] Verify communication between the ROS nodes and the Pixhawk
    * [x] Setup and test QGroundControl
    * [x] Test QGC with the telemetry radio
    * [x] Calibrate the flight controller
- [x] Verify drone sanity flight (manual mode, at ISR-Tagus and/or Cybaer-lab): 
    * [x] Gazebo simulation with the mission algorithm running on the onboard computer
    * [x] Indoor power on (no blades)
    * [x] Outdoor power on with blades (see if Pixhawk controller gains need to be changed)
- [x] Range test
- [x] Manual flight with data acquisition and ground monitoring
- [x] Dummy weight lifting tests (see if Pixhawk controller gains need to be changed) (no onboard computer installed) - create thrust curve.
- [x] PixHawk autonomous mission mode trials:
    * [x] Takeoff, hover, land (see if controller gains need to be changed)
    * [x] Typical mission for the controllers to be tested
    * [x] Off-board controller(s) running but not controlling vehicle (collected data used afterwards to see response of off-board controllers)
- [x] First off-board waypoint simple controller, no MPC, Guassian and/or GM;
- [ ] Online off-board MPC mission for PirePuma algorithms;
- [ ] Repeat previous trials with improved missions or algorithms.


