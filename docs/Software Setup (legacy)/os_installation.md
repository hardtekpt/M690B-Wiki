---
hide:
  - footer
---

# OS Installation and Setup

This guide provides installation instructions for the **JetPack** software which is used as an OS for the onboard computer. The following steps describe how to install the version *5.1.1* of **JetPack** on the onboard computer.  

## 1. SDK Manager

The installation of the operating system should be made using the [**NVIDIA SDK Manager**](https://docs.nvidia.com/sdk-manager/index.html) running on a computer with *Ubuntu 18.04* or *Ubuntu 20.04*. For reference, the installation was made using a machine with *Ubuntu 18.04* and the SDK Manager version *1.9.1.10899*. This will allow us to install the OS onto an NVME SSD instead of using a micro-SD card.

## 2. OS Installation

The SDK Manager can then be used to flash the *JetPack 5.1.1 (rev. 1)* OS onto the board as shown in the following images. Before flashing there is an option to select which storage solution to use for the OS as well as the ability to configure a user so as not to require a physical connection to the board on first boot to finish the OS installation.

![SDK Manager Setup](../assets/sdk_manager_1.png "SDK Manager Setup")
![Target Components Selection](../assets/sdk_manager_2.png "Target Components Selection")

## 3. Initial Setup

After the first boot it is recommended to update the installed packages and also to install some usefull additional packages:

```bash
sudo apt update
sudo apt upgrade
sudo apt install nano tmux python3-pip
```

Following that, *jtop* can be installed using pip. *jtop* is a tool that shows detailded stats and information about the nvidia board where it is running:

```bash
sudo pip3 install -U jetson-stats
```

## 4. Compiling OpenCV

Even though OpenCV comes installed with the *JetPack* OS, it does not take advantage of the GPU. To solve this, the script from [mdegans](https://github.com/mdegans/nano_build_opencv) was used and the version **4.6.0** of OpenCV can be installed:

```bash
git clone https://github.com/mdegans/nano_build_opencv.git
cd nano_build_opencv
chmod +x build_opencv.sh
./build_opencv.sh 4.6.0
```

!!! note

    This process takes a while. On the **NVIDIA Jetson Orin Nano** it takes approximately 2 hours.