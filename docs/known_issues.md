---
hide:
  - footer
---
# Known issues

## OS not booting into GUI

Sometimes when booting the onboard computer with a monitor and peripherals attached, the OS does not boot into the GUI but is still accessible through SSH. It was found that running the command `sudo systemctl set-default graphical.target` and rebooting seems to resolve this issue.