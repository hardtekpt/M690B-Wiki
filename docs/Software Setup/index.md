---
hide:
  - footer
---

# Software Setup

This section describes in detail all the procedures used to install and configure the software that runs on the vehicle's hardware components. Namely, the [OS and ROS installation on the onboard computer](https://hardtekpt.github.io/M690B-Wiki/Software%20Setup/os_installation/), the [VRPN and MAVROS Setup](https://hardtekpt.github.io/M690B-Wiki/Software%20Setup/vrpn_and_mavros/), the [PX4 Configuration Checklist](https://hardtekpt.github.io/M690B-Wiki/Software%20Setup/px4_configuration_checklist/), the [Drone Control Stack Setup](https://hardtekpt.github.io/M690B-Wiki/Software%20Setup/drone_control_stack/) and some [Usefull Packages](https://hardtekpt.github.io/M690B-Wiki/Software%20Setup/usefull_packages/).


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