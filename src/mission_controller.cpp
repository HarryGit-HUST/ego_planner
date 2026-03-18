#include "mission_controller.h"
#include "ego_planner_manager.h" // 包含你的 CEO
#include <cmath>
#include <tf/transform_datatypes.h>

MissionController::MissionController()
    : current_state_(MissionState::IDLE),
      is_connected_(false),
      has_global_plan_(false),
      init_yaw_(0.0) {}
MissionController::~MissionController() {}

// ============================================================================
// 1. 初始化：读 YAML、招小弟、设句柄
// ============================================================================
void MissionController::init(ros::NodeHandle &nh)
{
    // ==========================================================
    // [高阶工程规范]：异常捕获 (Try-Catch) 关键参数校验
    // ==========================================================
    try
    {
        std::vector<double> pt;

        // 读取任务一/二识别区坐标
        if (!nh.getParam("mission/wp_recognition", pt) || pt.size() < 2)
        {
            throw std::runtime_error("缺少关键航点参数: mission/wp_recognition");
        }
        param_.wp_recog = Eigen::Vector2d(pt[0], pt[1]);

        // 读取任务三/四空投区坐标
        if (!nh.getParam("mission/wp_airdrop", pt) || pt.size() < 2)
        {
            throw std::runtime_error("缺少关键航点参数: mission/wp_airdrop");
        }
        param_.wp_airdrop = Eigen::Vector2d(pt[0], pt[1]);

        // 读取任务五攻击区坐标
        if (!nh.getParam("mission/wp_strike", pt) || pt.size() < 2)
        {
            throw std::runtime_error("缺少关键航点参数: mission/wp_strike");
        }
        param_.wp_strike = Eigen::Vector2d(pt[0], pt[1]);

        // 非致命参数，给个默认值即可
        nh.param("mission/takeoff_height", param_.takeoff_height, 0.7);
    }
    catch (const std::exception &e)
    {
        // 一旦抛出异常，终端标红打印，并直接强行终止整个 ROS 节点！
        ROS_FATAL_STREAM("[Boss] 致命配置错误: " << e.what() << " (请检查 astar.yaml 文件是否加载正确！)");
        ros::shutdown();    // 优雅关闭 ROS
        exit(EXIT_FAILURE); // 强行退出进程
    }

    // ==========================================================
    // 实例化并初始化 CEO (PlannerManager)
    // ==========================================================
    planner_manager_ = std::make_shared<PlannerManager>();
    planner_manager_->init(nh);

    // ==========================================================
    // 保存 NodeHandle 到成员变量（供后续动态参数设置使用）
    // ==========================================================
    nh_ = nh;

    // ==========================================================
    // 设置 ROS 通信
    // ==========================================================
    state_sub_ = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &MissionController::stateCallback, this);
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &MissionController::odomCallback, this);
    setpoint_pub_ = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ROS_INFO("[Boss] 任务指挥官已就绪！各项比赛参数读取无误。");
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
        ROS_WARN_THROTTLE(0.5, "[IDLE] 尝试解锁和切换 OFFBOARD...");
        if (setOffboardAndArm())
        {
            // 1. 记录真实的物理起飞点
            init_pos_ = current_pos_;
            init_yaw_ = current_yaw_;



            // 3. 把相对航点转为世界绝对坐标
            param_.wp_recog += Eigen::Vector2d(init_pos_.x(), init_pos_.y());
            param_.wp_airdrop += Eigen::Vector2d(init_pos_.x(), init_pos_.y());
            param_.wp_strike += Eigen::Vector2d(init_pos_.x(), init_pos_.y());

            current_state_ = MissionState::TAKEOFF;
            ROS_INFO(">>> [任务开始] 起飞点已锁定，虚拟围墙已升起，开始爬升！");
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
            nh_.setParam("/pcl_enable", true);
            ROS_INFO("[Boss] PCL 已启用！");
             // 2. 【核心添加】Boss 下令：按照我现在的脚下位置，把虚拟围墙给我建起来！
            planner_manager_->buildWalls(init_pos_.x(), init_pos_.y());
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
        publishSetpoint(Eigen::Vector2d(init_pos_.x(), init_pos_.y()), 0.0, init_yaw_);
        // disarmDrone();
        ROS_INFO_THROTTLE(5.0, ">>> 任务已完成，等待裁判检查...");
        break;
    default:
        ROS_ERROR("未知的任务状态！");
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
    static ros::Time last_mode_req = ros::Time::now();
    static ros::Time last_arm_req = ros::Time::now();
    static bool has_sent_setpoint = false;
    
    // 调试输出：当前状态
    ROS_WARN_THROTTLE(0.5, "[setOffboardAndArm] mode=%s, armed=%d", 
                      mavros_state_.mode.c_str(), mavros_state_.armed);
    
    // 【关键】PX4 要求：切换 OFFBOARD 前必须先发送至少 1 秒的设定点
    if (!has_sent_setpoint)
    {
        // 先发送当前位置作为设定点
        publishSetpoint(Eigen::Vector2d(current_pos_.x(), current_pos_.y()), 
                        current_pos_.z(), current_yaw_);
        ROS_INFO_THROTTLE(0.5, "[setOffboardAndArm] 发送设定点以准备 OFFBOARD...");
        
        // 等待 1.5 秒后再尝试切换
        if (ros::Time::now() - last_mode_req > ros::Duration(1.5))
        {
            has_sent_setpoint = true;
        }
        return false;
    }
    
    // 【修改 1】OFFBOARD 模式请求
    if (mavros_state_.mode != "OFFBOARD" && (ros::Time::now() - last_mode_req > ros::Duration(1.0)))
    {
        mavros_msgs::SetMode srv;
        srv.request.custom_mode = "OFFBOARD";
        ROS_INFO("[setOffboardAndArm] 请求切换到 OFFBOARD 模式...");
        
        if (set_mode_client_.call(srv) && srv.response.mode_sent)
        {
            ROS_INFO("[setOffboardAndArm] OFFBOARD 切换成功！");
        }
        else
        {
            ROS_WARN("[setOffboardAndArm] OFFBOARD 切换失败，重试...");
        }
        last_mode_req = ros::Time::now();
    }
    
    // 【修改 2】解锁请求 - 使用独立的计时器，更频繁地尝试
    if (!mavros_state_.armed && (ros::Time::now() - last_arm_req > ros::Duration(0.5)))
    {
        mavros_msgs::CommandBool srv;
        srv.request.value = true;
        ROS_INFO("[setOffboardAndArm] 请求解锁...");
        
        if (arming_client_.call(srv) && srv.response.success)
        {
            ROS_INFO("[setOffboardAndArm] 解锁成功！");
        }
        else
        {
            ROS_WARN("[setOffboardAndArm] 解锁服务调用失败，请检查 PX4 安全条件");
        }
        last_arm_req = ros::Time::now();
    }

    // 【修改 3】只要解锁了就返回 true，让状态机进入 TAKEOFF
    return mavros_state_.armed;
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