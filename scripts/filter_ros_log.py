#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 日志过滤器
从 rosout.log 中提取干净的 ROS 日志信息（INFO/WARN/ERROR 等）
自动忽略：
- 时间戳数字
- 节点名中的乱码
- topics 列表
- tmux 等其他无关信息
"""

import re
import sys
import os
from pathlib import Path

# 日志级别正则匹配
LOG_PATTERN = re.compile(
    r'^[\d.]+\s+'  # 时间戳（忽略）
    r'(INFO|WARN|ERROR|DEBUG|FATAL)\s+'  # 日志级别
    r'(/\S+)\s+'  # 节点名
    r'\[([^\]]+)\]\s*'  # 文件位置（可选）
    r'\[topics:[^\]]*\]\s*'  # topics 列表（忽略）
    r'(.*)$'  # 实际消息
)

# 简化的日志级别匹配（无文件位置的情况）
SIMPLE_PATTERN = re.compile(
    r'^[\d.]+\s+'  # 时间戳（忽略）
    r'(INFO|WARN|ERROR|DEBUG|FATAL)\s+'  # 日志级别
    r'(/\S+)\s+'  # 节点名
    r'(.*)$'  # 实际消息
)

# 需要过滤的节点（tmux 等无关节点）
FILTER_NODES = [
    '/tmux',
    '/rosout',  # 可选：如果只想看自己节点的日志
]

# 需要过滤的消息关键词
FILTER_KEYWORDS = [
    'topics:',
    'Waiting for service',
    'waitForService',
]


def filter_log_line(line):
    """
    过滤并提取单行日志
    返回：(level, node, message) 或 None
    """
    line = line.strip()
    if not line:
        return None
    
    # 尝试完整匹配
    match = LOG_PATTERN.match(line)
    if match:
        level, node, filepath, message = match.groups()
        return (level, node, message.strip())
    
    # 尝试简化匹配
    match = SIMPLE_PATTERN.match(line)
    if match:
        level, node, message = match.groups()
        # 过滤掉 topics 信息
        if 'topics:' in message:
            return None
        return (level, node, message.strip())
    
    return None


def should_filter_out(level, node, message):
    """
    判断是否应该过滤掉这条日志
    """
    # 过滤指定节点
    for fn in FILTER_NODES:
        if node.startswith(fn):
            return True
    
    # 过滤指定关键词
    for kw in FILTER_KEYWORDS:
        if kw in message:
            return True
    
    return False


def format_output(level, node, message, color=True):
    """
    格式化输出日志
    """
    # 日志级别颜色
    colors = {
        'INFO': '\033[32m',    # 绿色
        'WARN': '\033[33m',    # 黄色
        'ERROR': '\033[31m',   # 红色
        'DEBUG': '\033[36m',   # 青色
        'FATAL': '\033[35m',   # 紫色
    }
    reset = '\033[0m'
    
    if color and level in colors:
        return f"{colors[level]}[{level}]{reset} {node} {message}"
    else:
        return f"[{level}] {node} {message}"


def main():
    # 默认日志路径
    log_path = os.path.expanduser('~/.ros/log/latest/rosout.log')
    
    # 支持命令行指定日志文件
    if len(sys.argv) > 1:
        log_path = sys.argv[1]
    
    if not os.path.exists(log_path):
        print(f"错误：日志文件不存在：{log_path}", file=sys.stderr)
        sys.exit(1)
    
    # 检测是否是终端输出（决定是否用颜色）
    use_color = sys.stdout.isatty()
    
    print(f"正在读取日志：{log_path}", file=sys.stderr)
    print("=" * 60, file=sys.stderr)
    
    with open(log_path, 'r', encoding='utf-8', errors='ignore') as f:
        for line in f:
            result = filter_log_line(line)
            if result:
                level, node, message = result
                if not should_filter_out(level, node, message):
                    print(format_output(level, node, message, use_color))


if __name__ == '__main__':
    main()
