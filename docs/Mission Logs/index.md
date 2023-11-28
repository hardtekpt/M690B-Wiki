---
hide:
  - footer
---

# Mission Logs

This section contains reports and logs from the missions executed with the M690B drone. These include *ulog* files from the flight computer, *rosbag* files from the onboard computer and video footage from the missions. Some useful tools for analysing the logs are: [PX4 Flight Review](https://review.px4.io/) and [Plotjuggler](https://plotjuggler.io/).

## Quickly generating mission log markdown files

To assist with the generation of mission log files, a python script is provided. The script uses a markdown template and replaces the variables with input from the user. In addition to generating the log file, the script also outputs to the terminal the entry that needs to added to the navigation section in the *mkdocs.yaml* file.

The script can be found inside the *mission_log_generator* folder and should be run with python3:

```bash
    chmod +x generate_markdown.py
    python3 generate_markdown.py
```

After running the script, just copy the generated file to the *Mission Logs* folder and add the entry to the *mkdocs.yaml* file