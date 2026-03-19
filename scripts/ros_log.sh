#!/bin/bash
#
# ROS 日志查看脚本
# 用法：
#   ./ros_log.sh                    # 查看所有日志
#   ./ros_log.sh -n /ego_planner_node  # 只看指定节点的日志
#   ./ros_log.sh -w                 # 只看 WARN 和 ERROR
#   ./ros_log.sh -f                 # 实时跟踪（类似 tail -f）
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FILTER_SCRIPT="${SCRIPT_DIR}/filter_ros_log.py"
LOG_FILE="$HOME/.ros/log/latest/rosout.log"

# 检查日志文件
if [ ! -f "$LOG_FILE" ]; then
    echo "错误：找不到日志文件 $LOG_FILE" >&2
    exit 1
fi

# 解析参数
NODE_FILTER=""
LEVEL_FILTER=""
FOLLOW_MODE=false

while getopts "n:wfe:" opt; do
    case $opt in
        n)
            NODE_FILTER="$OPTARG"
            ;;
        w)
            LEVEL_FILTER="WARN|ERROR|FATAL"
            ;;
        f)
            FOLLOW_MODE=true
            ;;
        e)
            # 自定义节点过滤（排除）
            EXCLUDE_NODE="$OPTARG"
            ;;
        \?)
            echo "用法：$0 [-n 节点名] [-w] [-f] [-e 排除节点]" >&2
            echo "  -n  只显示指定节点的日志" >&2
            echo "  -w  只显示 WARN/ERROR/FATAL" >&2
            echo "  -f  实时跟踪模式" >&2
            echo "  -e  排除指定节点" >&2
            exit 1
            ;;
    esac
done

# 实时跟踪模式
if [ "$FOLLOW_MODE" = true ]; then
    echo "实时跟踪日志中... (Ctrl+C 停止)" >&2
    tail -f "$LOG_FILE" | python3 "$FILTER_SCRIPT" /dev/stdin
    exit 0
fi

# 普通模式
if [ -n "$NODE_FILTER" ]; then
    # 按节点过滤
    python3 "$FILTER_SCRIPT" "$LOG_FILE" | grep -E "\[.*\] ${NODE_FILTER} "
elif [ -n "$LEVEL_FILTER" ]; then
    # 按级别过滤
    python3 "$FILTER_SCRIPT" "$LOG_FILE" | grep -E "^\[(WARN|ERROR|FATAL)\]"
elif [ -n "$EXCLUDE_NODE" ]; then
    # 排除指定节点
    python3 "$FILTER_SCRIPT" "$LOG_FILE" | grep -v "$EXCLUDE_NODE"
else
    # 显示全部
    python3 "$FILTER_SCRIPT" "$LOG_FILE"
fi
