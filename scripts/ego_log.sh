#!/bin/bash
#
# EGO Planner 专用日志查看脚本
# 只提取 /ego_planner_node 的日志，自动排除 /main 等节点
#
# 用法：
#   ./ego_log.sh                    # 查看 ego_planner 日志
#   ./ego_log.sh -w                 # 只看 WARN/ERROR
#   ./ego_log.sh -f                 # 实时跟踪
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
EXTRACT_SCRIPT="${SCRIPT_DIR}/extract_ego_planner_log.py"
LOG_FILE="$HOME/.ros/log/latest/rosout.log"

# 检查日志文件
if [ ! -f "$LOG_FILE" ]; then
    echo "错误：找不到日志文件 $LOG_FILE" >&2
    exit 1
fi

# 解析参数
LEVEL_FILTER=""
FOLLOW_MODE=false

while getopts "wf" opt; do
    case $opt in
        w)
            LEVEL_FILTER="warn"
            ;;
        f)
            FOLLOW_MODE=true
            ;;
        \?)
            echo "用法：$0 [-w] [-f]" >&2
            echo "  -w  只显示 WARN/ERROR/FATAL" >&2
            echo "  -f  实时跟踪模式" >&2
            exit 1
            ;;
    esac
done

# 实时跟踪模式
if [ "$FOLLOW_MODE" = true ]; then
    echo "实时跟踪 EGO Planner 日志... (Ctrl+C 停止)" >&2
    tail -f "$LOG_FILE" | python3 "$EXTRACT_SCRIPT" /dev/stdin
    exit 0
fi

# 普通模式
if [ "$LEVEL_FILTER" = "warn" ]; then
    # 只显示 WARN/ERROR/FATAL
    python3 "$EXTRACT_SCRIPT" "$LOG_FILE" 2>&1 | grep -E "^\[(WARN|ERROR|FATAL)\]"
else
    # 显示全部
    python3 "$EXTRACT_SCRIPT" "$LOG_FILE"
fi
