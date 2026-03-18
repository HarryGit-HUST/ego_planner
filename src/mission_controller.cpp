#include "mission_controller.h"
#include "ego_planner_manager.h" // 包含你的 CEO
#include <cmath>

MissionController::MissionController() : current_state_(MissionState::IDLE) {}
MissionController::~MissionController() {}

// ============================================================================
// 1. 初始化：读 YAML、招小弟、设句柄
// ============================================================================
void MissionController::init(ros::NodeHandle &nh)
{
    // 读取 YAML 里的比赛任务航点 (相对于起飞点的偏移量)
    nh.param("mission/takeoff_height", param_.takeoff_height, 1.2);

    std::vector<double> pt;
    if (nh.getParam("mission/wp_recognition", pt))
        param_.wp_recog = Eigen::Vector2d(pt[0], pt[1]);
    if (nh.getParam("mission/wp_airdrop", pt))
        param_.wp_airdrop = Eigen::Vector2d(pt[0], pt[1]);
    if (nh.getParam("mission/wp_strike", pt))
        param_.wp_strike = Eigen::Vector2d(pt[0], pt[1]);

    // 实例化并初始化 CEO (PlannerManager)
    planner_manager_ = std::make_shared<PlannerManager>();
    planner_manager_->init(nh);

    // 设置 ROS 通信
    state_sub_ = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &MissionController::stateCallback, this);
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &MissionController::odomCallback, this);
    setpoint_pub_ = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ROS_INFO("[Boss] 任务指挥官已就绪！");
}

// ============================================================================
// 2. 神级封装：将 EGO-Planner 避障浓缩为一个动作函数
// ============================================================================
bool MissionController::flyToXY(const Eigen::Vector2d &target_xy)
{
    Eigen::Vector2d curr_xy(current_pos_.x(), current_pos_.y());
    double dist_to_goal = (curr_xy - target_xy).norm();

    // 1. 到达判定 (阈值 0.2m)
    if (dist_to_goal < 0.2)
    {
        return true;
    }

    // 2. 检查是否需要重新规划 (没轨迹，或者前方轨迹突然出现新障碍物)
    bool need_replan = !has_global_plan_ || planner_manager_->checkCollision();

    if (need_replan)
    {
        if (planner_manager_->replan(curr_xy, target_xy))
        {
            has_global_plan_ = true;
        }
        else
        {
            ROS_WARN_THROTTLE(1.0, "[Boss] 寻路失败！原地悬停等待路径...");
            has_global_plan_ = false;
            // 失败时，输出当前位置悬停保命
            publishSetpoint(curr_xy, param_.takeoff_height, current_yaw_);
            return false;
        }
    }

    // 3. 正常循迹：向 CEO 索要当前时间戳应该在的绝对位置
    double t_sec = (ros::Time::now() - planner_manager_->getTrajStartTime()).toSec();
    Eigen::Vector2d cmd_pos = planner_manager_->getPosition(t_sec);

    // 4. 发送给飞控 (保持固定高度和机头朝向)
    publishSetpoint(cmd_pos, param_.takeoff_height, init_yaw_);

    return false; // 还没到终点
}

// ============================================================================
// 3. 主状态机：比赛流程核心 (tick 20Hz)
// ============================================================================
void MissionController::tick()
{
    // 安全防线：确保有数据
    if (!is_connected_)
        return;

    switch (current_state_)
    {

    // -----------------------------------------------------------
    case MissionState::IDLE:
        if (setOffboardAndArm())
        {
            // 【核心：记录 Init XYZ】 只有在切 OFFBOARD 解锁成功的瞬间，才记录绝对零点
            init_pos_ = current_pos_;
            init_yaw_ = current_yaw_;

            // 把 YAML 里的相对航点，加上真正的物理起点，变成世界绝对坐标
            param_.wp_recog += Eigen::Vector2d(init_pos_.x(), init_pos_.y());
            param_.wp_airdrop += Eigen::Vector2d(init_pos_.x(), init_pos_.y());
            param_.wp_strike += Eigen::Vector2d(init_pos_.x(), init_pos_.y());

            current_state_ = MissionState::TAKEOFF;
            ROS_INFO(">>> [任务开始] 起飞点已锁定，开始爬升！");
        }
        break;

    // -----------------------------------------------------------
    case MissionState::TAKEOFF:
        publishSetpoint(Eigen::Vector2d(init_pos_.x(), init_pos_.y()), param_.takeoff_height, init_yaw_);

        // 爬升到指定高度误差 < 0.15m
        if (std::abs(current_pos_.z() - param_.takeoff_height) < 0.15)
        {
            current_state_ = MissionState::NAV_RECOG_AREA;
            has_global_plan_ = false;
            ROS_INFO(">>> [任务一] 爬升完毕，前往目标识别区！");
        }
        break;

    // -----------------------------------------------------------
    case MissionState::NAV_RECOG_AREA:
        // flyToXY 内部会自动躲避抽签摆放的障碍1和障碍2
        if (flyToXY(param_.wp_recog))
        {
            current_state_ = MissionState::HOVER_RECOGNIZE;
            hover_start_time_ = ros::Time::now(); // 开始计时
            ROS_INFO(">>> [任务二] 到达识别区，开始 3 秒悬停识别...");
        }
        break;

    // -----------------------------------------------------------
    case MissionState::HOVER_RECOGNIZE:
        // 强制将 Z 轴锁定在 takeoff_height，MAVROS 位置控制可轻松保证垂直偏差 < 3cm
        publishSetpoint(param_.wp_recog, param_.takeoff_height, init_yaw_);

        if ((ros::Time::now() - hover_start_time_).toSec() > 3.0)
        {
            // TODO: 触发图像识别载荷，记录识别结果
            // triggerRecognition();

            current_state_ = MissionState::NAV_AIRDROP_AREA;
            has_global_plan_ = false;
            ROS_INFO(">>> [任务三] 识别完毕！绕过圆柱前往投放区...");
        }
        break;

    // -----------------------------------------------------------
    case MissionState::NAV_AIRDROP_AREA:
        // flyToXY 自动绕过任务三的圆柱形障碍物
        if (flyToXY(param_.wp_airdrop))
        {
            current_state_ = MissionState::HOVER_AIRDROP;
            hover_start_time_ = ros::Time::now();
            ROS_INFO(">>> [任务四] 到达投放区上方，悬停准备投掷...");
        }
        break;

    // -----------------------------------------------------------
    case MissionState::HOVER_AIRDROP:
        publishSetpoint(param_.wp_airdrop, param_.takeoff_height, init_yaw_);

        // TODO: 调用视觉识别投放标 (可用 YOLO 伺服对准，微调 setpoint_raw)
        // if (is_aligned) { dropPayload(); state = NAV_STRIKE_AREA; }

        // 临时用定时器替代
        if ((ros::Time::now() - hover_start_time_).toSec() > 3.0)
        {
            // TODO: 舵机开锁，投放物资
            // triggerAirdropServo();

            current_state_ = MissionState::NAV_STRIKE_AREA;
            has_global_plan_ = false;
            ROS_INFO(">>> [任务五] 投掷完毕！前往靶标攻击方位...");
        }
        break;

    // -----------------------------------------------------------
    case MissionState::NAV_STRIKE_AREA:
        if (flyToXY(param_.wp_strike))
        {
            current_state_ = MissionState::LASER_STRIKE;
            hover_start_time_ = ros::Time::now();
            ROS_INFO(">>> [任务五] 抵达攻击阵位，准备激光打击！");
        }
        break;

    // -----------------------------------------------------------
    case MissionState::LASER_STRIKE:
        publishSetpoint(param_.wp_strike, param_.takeoff_height, init_yaw_);

        // TODO: 开启机载激光器，利用云台或机身偏航瞄准
        // triggerLaser();

        if ((ros::Time::now() - hover_start_time_).toSec() > 2.0)
        {
            // 裁判确认声光装置触发后
            current_state_ = MissionState::RETURN_TO_LAUNCH;
            has_global_plan_ = false;
            ROS_INFO(">>> [任务六] 攻击成功！全速返航！");
        }
        break;

    // -----------------------------------------------------------
    case MissionState::RETURN_TO_LAUNCH:
        // 目标点设为初始记录的 X Y 坐标
        if (flyToXY(Eigen::Vector2d(init_pos_.x(), init_pos_.y())))
        {
            current_state_ = MissionState::LANDING;
            ROS_INFO(">>> [任务六] 回到起飞点上空，开始降落...");
        }
        break;

    // -----------------------------------------------------------
    case MissionState::LANDING:
        // 持续降低 Z 轴
        publishSetpoint(Eigen::Vector2d(init_pos_.x(), init_pos_.y()), current_pos_.z() - 0.2, init_yaw_);

        if (current_pos_.z() < init_pos_.z() + 0.1)
        {
            current_state_ = MissionState::FINISHED;
            ROS_INFO(">>> 比赛完美收官！停桨！");
        }
        break;

    // -----------------------------------------------------------
    case MissionState::FINISHED:
        // TODO: 发送 MAVROS 锁定命令 (Disarm)
        setpoint_raw.type_mask = 0;
        // disarmDrone();
        ROS_INFO_THROTTLE(5.0, ">>> 任务已完成，等待裁判检查...");
        break;
    }
}

// ============================================================================
// 辅助函数
// ============================================================================
void MissionController::publishSetpoint(const Eigen::Vector2d &xy, double z, double yaw)
{
    mavros_msgs::PositionTarget msg;
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask = 0b101111111000; // 只控制 XYZ 和 Yaw
    msg.position.x = xy.x();
    msg.position.y = xy.y();
    msg.position.z = z;
    msg.yaw = yaw;
    setpoint_pub_.publish(msg);
}

bool MissionController::setOffboardAndArm()
{
    static ros::Time last_req = ros::Time::now();
    if (mavros_state_.mode != "OFFBOARD" && (ros::Time::now() - last_req > ros::Duration(1.0)))
    {
        mavros_msgs::SetMode srv;
        srv.request.custom_mode = "OFFBOARD";
        set_mode_client_.call(srv);
        last_req = ros::Time::now();
    }
    else if (!mavros_state_.armed && (ros::Time::now() - last_req > ros::Duration(1.0)))
    {
        mavros_msgs::CommandBool srv;
        srv.request.value = true;
        arming_client_.call(srv);
        last_req = ros::Time::now();
    }
    return (mavros_state_.mode == "OFFBOARD" && mavros_state_.armed);
}

void MissionController::stateCallback(const mavros_msgs::State::ConstPtr &msg)
{
    mavros_state_ = *msg;
    is_connected_ = msg->connected;
}

void MissionController::odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_pos_.x() = msg->pose.pose.position.x;
    current_pos_.y() = msg->pose.pose.position.y;
    current_pos_.z() = msg->pose.pose.position.z;

    // 从四元数解算 Yaw... (省略常规代码)
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    double r, p;
    tf::Matrix3x3(q).getRPY(r, p, current_yaw_);
}