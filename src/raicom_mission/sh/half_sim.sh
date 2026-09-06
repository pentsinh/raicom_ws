#!/bin/bash
tmux kill-session -t sim
tmux new -d -s sim

## 分割成四块
tmux split-window -h -t sim
tmux split-window -h -t sim.0
tmux split-window -v -t sim.0
tmux split-window -v -t sim.2
tmux split-window -h -t sim.2

## 填入指令 (自动处理 cd, source 和 roscore 等待)
tmux send -t sim.0 "sleep 1 && roslaunch --screen foxglove_bridge foxglove_bridge.launch port:=8765" C-m 
tmux send -t sim.1 "sleep 2 && sudo modprobe v4l2loopback devices=2 video_nr=10,12 card_label=\"FakeCam10,FakeCam12\" exclusive_caps=1 && ffmpeg -re -loop 1 -i ./Downloads/laser_target.jpg -vf \"scale=640:480:force_original_aspect_ratio=decrease,pad=640:480:(ow-iw)/2:(oh-ih)/2\" -r 30 -f v4l2 -pix_fmt yuyv422 /dev/video12" C-m
tmux send -t sim.2 'sleep 3 && ffmpeg -re -loop 1 -i ./Downloads/drop_target.jpg -vf "scale=640:480:force_original_aspect_ratio=decrease,pad=640:480:(ow-iw)/2:(oh-ih)/2" -r 30 -f v4l2 -pix_fmt yuyv422 /dev/video10' C-m
tmux send -t sim.3 'sleep 4 && roslaunch raicom_mission scene_generator.launch' C-m 
tmux send -t sim.4 'sleep 5 && roslaunch raicom_mission arduino_nano.launch' C-m
tmux send -t sim.5 'sleep 6 && roslaunch raicom_mission multy_cam.launch' C-m
## 显示刚刚创建的会话
tmux a -t sim
