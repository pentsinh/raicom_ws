#!/bin/bash
tmux kill-session -t start
tmux new -d -s start

## 分割成四块
tmux split-window -h -t start
tmux split-window -h -t start.0
tmux split-window -v -t start.0
tmux split-window -v -t start.2

## 填入指令 (自动处理 cd, source 和 roscore 等待)
tmux send -t start.0 "sleep 1;source ~/raicom_ws/devel/setup.zsh;roslaunch bringup location.launch" C-m 
tmux send -t start.1 "sleep 2;source ~/raicom_ws/devel/setup.zsh;roslaunch raicom_mission start_yolo.launch" C-m
tmux send -t start.2 "sleep 2;source ~/raicom_ws/devel/setup.zsh;rostopic echo /mavros/local_position/pose" C-m
# tmux send -t start.2 'sleep 3;source ~/raicom_ws/devel/setup.zsh;roslaunch raicom_mission multy_cam.launch' C-m
tmux send -t start.3 'sleep 3;source ~/raicom_ws/devel/setup.zsh;roslaunch raicom_mission map_accumulator.launch' C-m
tmux send -t start.4 'sleep 4;source ~/raicom_ws/devel/setup.zsh;roslaunch raicom_mission raicom_mission.launch' C-m 
# tmux send -t start.4 'sleep 5;source ~/raicom_ws/devel/setup.zsh;roslaunch complete_mission ego_planner_mid360.launch' C-m
## 显示刚刚创建的会话
tmux a -t start.4
