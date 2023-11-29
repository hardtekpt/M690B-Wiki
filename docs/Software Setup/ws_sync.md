---
hide:
  - footer
---
# Workspace Syncronization

A python script is available in the root of this repository to fetch all changes to the configured repositories. This configuration is made through a yaml file called `packages_per_drone.yaml` as shown in the following example:

```yaml
    - name: m690b_ist
      base_path: /home/helios/uav/uav_ws/src/
      packages:
        - name: drone_control_stack
          repo: https://github.com/hardtekpt/drone_control_stack
        - name: mav_tools
          repo: https://github.com/hardtekpt/mav_tools
    - name: m690b_nova
      base_path: /home/icarus/uav/uav_ws/src/
      packages:
        - name: drone_control_stack
          repo: https://github.com/hardtekpt/drone_control_stack
```

By changing this file with any repos that are to be included in either drone's workspace, the python script will automatically clone and update the packages whenever called. This process can be further automated if the script is called when an action is triggered. For example, on every boot, or whenever the drone has access to the Internet.

To manually run script use:

```bash
    chmod +x sync_ws.py
    python3 sync_ws.py
```

To automatically run the script on boot open crontab (`crontab -e`) and add the line:

```bash
    @reboot python3 <path to wiki repot>/sync_ws.py
```