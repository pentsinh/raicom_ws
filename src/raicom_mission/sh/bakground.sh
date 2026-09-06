#!/bin/bash
tmux kill-session -t background
tmux new -d -s background

## 分割成四块
tmux split-window -h -t background
tmux split-window -h -t background.0
tmux split-window -v -t background.0


## 填入指令 (自动处理 cd, source 和 roscore 等待)
tmux send -t background.0 "sleep 1 &&  roslaunch --screen foxglove_bridge foxglove_bridge.launch port:=8765" C-m
tmux send -t background.1 'sleep 2 && roslaunch raicom_mission multy_cam.launch' C-m
tmux send -t background.2 'sleep 3 && stty -F /dev/ttyUSB0 -hupcl' C-m
tmux send -t background.3 'sleep 4 && roslaunch raicom_mission arduino_nano.launch' C-m


## 显示刚刚创建的会话
tmux a -t background
