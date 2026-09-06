#!/bin/bash
tmux kill-session -t sim
tmux new -d -s sim

## 分割成四块
tmux split-window -h -t sim
tmux split-window -h -t sim.0
tmux split-window -v -t sim.0


## 填入指令 (自动处理 cd, source 和 roscore 等待)
tmux send -t sim.0 "sleep 2 && sudo modprobe v4l2loopback devices=2 video_nr=10,12 card_label=\"FakeCam10,FakeCam12\" exclusive_caps=1 && ffmpeg -re -loop 1 -i ./Downloads/laser_target.jpg -vf \"scale=640:480:force_original_aspect_ratio=decrease,pad=640:480:(ow-iw)/2:(oh-ih)/2\" -r 30 -f v4l2 -pix_fmt yuyv422 /dev/video12" C-m
tmux send -t sim.1 'sleep 3 && ffmpeg -re -loop 1 -i ./Downloads/drop_target.jpg -vf "scale=640:480:force_original_aspect_ratio=decrease,pad=640:480:(ow-iw)/2:(oh-ih)/2" -r 30 -f v4l2 -pix_fmt yuyv422 /dev/video10' 
tmux send -t sim.2 'sleep 4 && roslaunch raicom_mission sim.launch' C-m 
tmux send -t sim.3 'sleep 6 && roslaunch raicom_mission multy_cam.launch' C-m




tmux new-window -t sim -n ctl

## 分割成四块
tmux split-window -h -t ctl
tmux split-window -h -t ctl.0
tmux split-window -v -t ctl.0
# tmux split-window -v -t ctl.2

## 填入指令 (自动处理 cd, source 和 roscore 等待)
tmux send -t ctl.0 "sleep 2 && roslaunch raicom_mission start_yolo.launch" C-m
tmux send -t ctl.1 'sleep 3 && roslaunch ring_detector map_accumulator.launch' C-m
tmux send -t ctl.2 'sleep 4 && roslaunch raicom_mission raicom_mission.launch' C-m 
tmux send -t ctl.3 'sleep 5 && roslaunch complete_mission ego_planner_mid360.launch' C-m
## 显示刚刚创建的会话
tmux a -t sim
