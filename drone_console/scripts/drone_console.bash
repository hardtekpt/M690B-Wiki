#! /bin/bash

sessname="drone_console"
window0name="console"
window1name="extra"

basepath="/home/jetson/scripts"
wspath="/home/jetson/uav/uav_ws"
bagfilespath="/home/jetson/bagfiles"

tmux new -s $sessname -n $window0name -d
tmux source-file ~/.tmux.conf

tmux split-window -h -p 85 -t "$sessname"

tmux send-keys -t "$sessname" "source /opt/ros/noetic/setup.bash" Enter
tmux send-keys -t "$sessname" "cd $wspath" Enter
tmux send-keys -t "$sessname" "source devel/setup.bash" Enter
tmux send-keys -t "$sessname" "clear" Enter

tmux split-window -v -p 10 -t "$sessname"

tmux send-keys -t "$sessname" "source /opt/ros/noetic/setup.bash" Enter
tmux send-keys -t "$sessname" "cd $wspath/src/drone_bringup/launch" Enter
tmux send-keys -t "$sessname" "clear" Enter
tmux send-keys -t "$sessname" "nano drone_sim.launch" Enter

tmux select-pane -t 0
tmux split-window -v -p 93 -t "$sessname"

tmux send-keys -t "$sessname" "$basepath/mavros_run.bash" Enter

tmux select-pane -t 0
tmux send-keys -t "$sessname" "source /opt/ros/noetic/setup.bash" Enter
tmux send-keys -t "$sessname" "cd $wspath" Enter
tmux send-keys -t "$sessname" "source devel/setup.bash" Enter
tmux send-keys -t "$sessname" "rosrun mav_tools mav_tools.py 'm690b1/'" Enter

tmux new-window -n $window1name
tmux send-keys -t "$sessname" "cd $bagfilespath && ls -l" Enter
tmux split-window -h -p 60 -t "$sessname" 
tmux send-keys -t "$sessname" "cd ~ && source /opt/ros/noetic/setup.bash && source $wspath/devel/setup.bash && clear" Enter

tmux select-window -t 0