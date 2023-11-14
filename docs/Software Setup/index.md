---
hide:
  - footer
---

# Software Setup

This section describes in detail all the procedures used to install and configure the software that runs on the vehicle's hardware components. Namely, the [OS installation and Setup](os_installation), the [ROS and MAVROS installation](ros_installation), the [PX4 Configuration Checklist](px4_configuration_checklist), the [Drone Control Stack Setup](drone_control_stack) and the [Drone Console](drone_console).

``` mermaid
graph LR
  A(MOCAP) -- UDP --> B(ROUTER)
  B -- UDP --> C(vrpn_client_ros)
  subgraph Onboard Computer
    C -- ROSTOPIC --- D(MAVROS)
    G("Main Algorithm (Mission)") -- ROSTOPIC --- D
  end
  subgraph Flight Computer
    D -- MAVLINK --- E(PX4-Autopilot)
  end
  B -- "UDP (gcs_url)" --- D
  subgraph Companion Computer
    F(QGroundControl)
  end
  F -- "UDP (fcu_url)" --- B
  F-. "TELEM RADIO" .- E
```