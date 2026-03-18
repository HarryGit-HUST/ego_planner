#!/bin/bash

# Session 名称
SESSION="ego_planner_session"

# ================= 配置路径 =================
MAIN_WS=~/first_task_ws
PX4_PATH=/home/jetson/Libraries/PX4-Autopilot

# 清理旧环境
tmux kill-session -t $SESSION 2>/dev/null
sleep 1

# ====================================================
# 窗口 0: 基础仿真 (Sim + Core)
# ====================================================
tmux new-session -d -s $SESSION -n "sim_core"

# Pane 0.0: roscore
tmux send-keys -t $SESSION:0.0 'roscore' C-m
sleep 2

# Pane 0.1: Gazebo 仿真
tmux split-window -h -t $SESSION:0
tmux send-keys -t $SESSION:0.1 "sleep 3; \
source ${MAIN_WS}/devel/setup.bash; \
source ${PX4_PATH}/Tools/setup_gazebo.bash ${PX4_PATH} ${PX4_PATH}/build/px4_sitl_default; \
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:${PX4_PATH}:${PX4_PATH}/Tools/sitl_gazebo; \
roslaunch tutorial_gazebo sim.launch" C-m
# ====================================================
# 窗口 1: PCL 感知
# ====================================================
tmux new-window -t $SESSION:1 -n "pcl_perception2"
tmux send-keys -t $SESSION:1 "sleep 10; source ${MAIN_WS}/devel/setup.bash; cd ${MAIN_WS}/src/pcl_detection2/shell; bash pcl_detection.sh" C-m

# ====================================================
# 窗口 2: 任务控制与监控 (Mission + Monitor)
# ====================================================
tmux new-window -t $SESSION:2 -n "mission_ctrl"

# Pane 2.0: 话题监控 - 位置信息
tmux send-keys -t $SESSION:2.0 "sleep 5; rostopic echo /mavros/local_position/pose" C-m

# Pane 2.1: EGO-Planner 主控节点 (含 PCL 点云处理)
tmux split-window -v -t $SESSION:2.0
tmux send-keys -t $SESSION:2.1 "sleep 15; source ${MAIN_WS}/devel/setup.bash; roslaunch ego_planner ego_planner.launch" C-m


# ====================================================
# 收尾
# ====================================================
tmux select-layout -t $SESSION:2 tiled
tmux select-window -t $SESSION:2
tmux select-pane -t $SESSION:2.1
tmux attach-session -t $SESSION
