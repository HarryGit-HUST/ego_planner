# ROS 日志过滤工具

## 快速使用

### 通用日志查看

```bash
# 查看所有清理后的日志（去掉时间戳、topics 等杂乱信息）
./scripts/ros_log.sh

# 只看特定节点的日志
./scripts/ros_log.sh -n /ego_planner_node
./scripts/ros_log.sh -n /mavros
./scripts/ros_log.sh -n /gazebo

# 只看警告和错误
./scripts/ros_log.sh -w

# 实时跟踪日志（类似 tail -f）
./scripts/ros_log.sh -f

# 排除特定节点
./scripts/ros_log.sh -e /rosout
```

### EGO Planner 专用日志

```bash
# 只看 /ego_planner_node 的日志（自动排除 /main 等节点）
./scripts/ego_log.sh

# 只看 WARN/ERROR
./scripts/ego_log.sh -w

# 实时跟踪
./scripts/ego_log.sh -f
```

## 功能特点

✅ **自动过滤**：
- 时间戳数字
- `[topics: ...]` 列表
- tmux 等无关节点
- 等待服务的重复信息

✅ **彩色输出**（终端模式下）：
- 🟢 INFO - 绿色
- 🟡 WARN - 黄色
- 🔴 ERROR - 红色

✅ **灵活过滤**：
- 按节点名称过滤
- 按日志级别过滤
- 实时跟踪模式

## 示例输出

### 原始日志
```
1773913641.827680920  Node Startup
0.000000000 INFO /vehicle_spawn_harry_Legion_R7000P_AHP9_9615_712277959287211163
7 [gazebo_interface.py:19(spawn_sdf_model_client)] [topics: /rosout, /clock] Cal
ling service /gazebo/spawn_sdf_model
```

### 过滤后
```
[INFO] /vehicle_spawn_harry_Legion_R7000P_AHP9_9615_7122779592872111637 Calling service /gazebo/spawn_sdf_model
```

## 文件说明

| 文件 | 说明 |
|------|------|
| `filter_ros_log.py` | Python 日志过滤核心脚本 |
| `ros_log.sh` | Bash 包装脚本，提供便捷参数（通用） |
| `extract_ego_planner_log.py` | EGO Planner 专用日志提取器 |
| `ego_log.sh` | EGO Planner 专用 Bash 包装脚本 |

## 依赖

- Python 3
- ROS Noetic
