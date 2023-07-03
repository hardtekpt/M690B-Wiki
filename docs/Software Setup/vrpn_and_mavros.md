---
hide:
  - footer
---

# VRPN and MAVROS Setup

## VRPN

In order to access data from the **Optitrack** MOCAP system the [vrpn_client_ros](http://wiki.ros.org/vrpn_client_ros) package was used.

```bash
sudo apt install ros-noetic-vrpn-client-ros
```

This package listens on the IP of the computer running the MOCAP sosftware and introduces pose information of the captured rigid bodies into ros topics. To do this a launch file is provided:

```bash
roslaunch vrpn_client_ros sample.launch
```

In this launch file, the available parameters were configured as such (The IP used corresponds to the MOCAP system of the 8th floor at ISR-Alameda):

```yaml
server: 192.168.0.2
port: 3883
update_frequency: 30.0
frame_id: world
use_server_time: false
broadcast_tf: true
refresh_tracker_frequency: 0.2
```

## MAVROS

To communicate with the flight computer (running the **PX4** autopilot) using the **MAVLink** protocol through ROS, the [**MAVROS**](http://wiki.ros.org/mavros) package was used. It is assumed that the flight computer is connected to the onboard computer through USB.

```bash
sudo apt install ros-noetic-mavros
sudo apt install ros-noetic-mavros-extras
cd /opt/ros/noetic/lib/mavros
sudo ./install_geographiclib_datasets.sh 
```

Additionally, to allow access to the USB ports for serial communication with the flight computer, these commands are needed:

```bash
sudo gpasswd -a $USER dialout
sudo reboot now
```

The package can now be tested by running `roslaunch mavros px4.launch` and verifying that data is being published in the available ROS topics (`rostopic list` and `rostopic echo ...`).

The **MAVROS** package also allows to redirect the **MAVLink** communications to another computer, where [QGroundControl](http://qgroundcontrol.com/) can be used for example. To do this, an option can be included when calling the launch file - `roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600 gcs_url:=udp://@${GCS_IP}:${GCS_PORT}`, where *fcu_url* is where the flight computer is connected, *GCS_IP* is the IP of the computer that wants to access the **MAVLink** communications and *GCS_Port* is an arbitrary port.

### MAVROS Example

1.  Onboard computer IP: 192.168.1.126; 
2.  Flight computer connected to onboard computer through USB (/dev/ttyACM0);
3.  Other computer IP: 192.168.1.118;
4.  Run `roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:57600 gcs_url:=udp://@192.168.1.118:15001` on the onboard computer;
5.  Run `roslaunch mavros px4.launch fcu_url:=udp://:15001@ gcs_url:=udp://@localhost:15002` on the other computer;
6.  `rostopic list` and `rostopic echo ...` on the other computer will show the data from the flight computer;
7.  QGroundControl can be used on the other computer on port 15002.

