---
hide:
  - footer
---

# Software Setup V2

This section describes in detail all the procedures used to install and configure the software that runs on the vehicle's hardware components. 

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