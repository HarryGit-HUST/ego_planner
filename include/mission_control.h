#ifndef MISSION_CONTROLLER_H
#define MISSION_CONTROLLER_H

// 1. 标准库
#include <memory> // 为了使用 std::shared_ptr (智能指针，用来雇佣下属部门)
#include <string>
#include <vector>

// 2. ROS 与 数学基础
#include <ros/ros.h>
#include <Eigen/Dense> // 统一使用 Eigen::Vector3d 表示空间坐标

// 3. MAVROS 相关的消息类型 (Boss 需要和飞控直接对话)
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Odometry.h>

// 4. 前向声明 (Forward Declaration) - 极其高级的 C++ 技巧！
// 告诉 Boss：“我们公司有 PlannerManager 这么个部门，具体它是干啥的你现在别管，以后会给你介绍。”
// 这样可以避免在这里 #include "planner_manager.h"，极大加快编译速度！
class PlannerManager;
// 任务状态机
enum class MissionState
{
    IDLE,            // 地面怠速，等待解锁
    TAKEOFF,         // 起飞爬升
    WAYPOINT_NAV,    // 航点避障飞行 (呼叫 CEO PlannerManager)
    VISION_ACCEPT,     // 视觉识别目标，等待结果
    TURN_1,           // 第一次转向，调整姿态
    THROW_MISSION,       // 投掷任务，飞行到指定位置投掷物品
    VISION_DETECT,    //视觉识别靶标，准备击穿
    LANDING_DESCEND, // 垂直降落
    FINISHED         // 任务结束，停桨
};
class MissionController
{
public:
    // 构造与析构
    MissionController();
    ~MissionController();

    // 1. 初始化函数：只在程序刚启动时运行一次。
    // 负责：读取 YAML 参数，注册 ROS 订阅器/发布器，实例化下属部门。
    void init(ros::NodeHandle &nh);

    // 2. 主循环滴答函数：在 main 的 while 循环里以 20Hz 频率被疯狂调用。
    // 负责：推动状态机流转。
    void tick();

private:
    // ================= 内部属性 =================
    MissionState current_state_;  // 当前状态
    Eigen::Vector3d current_pos_; // 当前飞机绝对位置
    double current_yaw_;          // 当前机头朝向
    bool is_armed_;               // 是否解锁
    std::string current_mode_;    // 当前飞行模式 (如 "OFFBOARD")

    // 从 YAML 读进来的参数写进一个内部结构体，保持整洁
    struct Param
    {
        double takeoff_height;
        std::vector<Eigen::Vector3d> waypoints; // 航点列表
        // ... 降落等其他参数
    } param_;

    // 当前正在飞第几个航点
    int current_wp_index_;

    // ================= ROS 通信句柄 =================
    ros::Subscriber state_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher setpoint_pub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;

    // ================= 雇佣下属部门 (依赖注入核心) =================
    // 使用 shared_ptr 管理 CEO，Boss 随时可以把终点发给 CEO 让他算轨迹
    std::shared_ptr<PlannerManager> planner_manager_;

    // ================= 内部工作手册 (私有辅助函数) =================

    // ROS 回调函数
    void stateCallback(const mavros_msgs::State::ConstPtr &msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

    // 状态机行为函数 (拆分 tick，让代码极其清爽)
    void handleIdle();
    void handleTakeoff();
    void handleWaypointNav();
    // void handleVisionCross(); ...

    // 通用飞控动作
    bool setOffboardAndArm();
};

#endif // MISSION_CONTROLLER_H