---
hide:
  - footer
---

# ROS Installation

The version of the installed OS is based on *Ubuntu 20.04* for which the latest version of ROS1 is *ROS Noetic*. To install it, the official guide in the [ROS wiki](http://wiki.ros.org/noetic/Installation/Ubuntu) was followed.


1.  Setup sources.list
    ```bash
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    ```

2.  Setup keys
    ```bash
    sudo apt install curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    ```

3.  Installation (the desktop-full variant was selected for this specific installation)
    ```bash
    sudo apt install ros-noetic-desktop-full
    ```

4.  Environment setup
    ```bash
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

5.  Dependencies for building packages
    ```bash
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

    sudo rosdep init
    rosdep update
    ```

# MAVROS

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

