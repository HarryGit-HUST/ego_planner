#include "mission_controller.h"
#include "ego_planner_manager.h"
#include <cmath>
#include <functional>
#include <tf/transform_datatypes.h>

MissionController::MissionController()
    : current_state_(MissionState::IDLE),
      is_connected_(false),
      has_global_plan_(false),
      init_yaw_(0.0),
      is_planning_(false) {} // 初始化原子变量

MissionController::~MissionController() {}

void MissionController::init(ros::NodeHandle &nh)
{
    try
    {
        std::vector<double> pt;
        if (!nh.getParam("mission/wp_recognition", pt) || pt.size() < 2)
            throw std::runtime_error("缺少 wp_recognition");
        param_.wp_recog = Eigen::Vector2d(pt[0], pt[1]);

        if (!nh.getParam("mission/wp_airdrop", pt) || pt.size() < 2)
            throw std::runtime_error("缺少 wp_airdrop");
        param_.wp_airdrop = Eigen::Vector2d(pt[0], pt[1]);

        if (!nh.getParam("mission/wp_strike", pt) || pt.size() < 2)
            throw std::runtime_error("缺少 wp_strike");
        param_.wp_strike = Eigen::Vector2d(pt[0], pt[1]);

        nh.param("mission/takeoff_height", param_.takeoff_height, 1.2);
    }
    catch (const std::exception &e)
    {
        ROS_FATAL_STREAM("[Boss] 致命配置错误: " << e.what());
        ros::shutdown();
        exit(EXIT_FAILURE);
    }

    planner_manager_ = std::make_shared<PlannerManager>();
    planner_manager_->init(nh);

    nh_ = nh;
    state_sub_ = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &MissionController::stateCallback, this);
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &MissionController::odomCallback, this);
    setpoint_pub_ = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    ROS_INFO("[Boss] 任务指挥官已就绪！各项比赛参数读取无误。");
}


void MissionController::tick()
{
    if (!is_connected_)
        return;

    switch (current_state_)
    {

    case MissionState::IDLE:
        ROS_WARN_THROTTLE(0.5, "[IDLE] 尝试解锁和切换 OFFBOARD...");
        if (setOffboardAndArm())
        {
            init_pos_ = current_pos_;
            init_yaw_ = current_yaw_;

            param_.wp_recog += Eigen::Vector2d(init_pos_.x(), init_pos_.y());
            param_.wp_airdrop += Eigen::Vector2d(init_pos_.x(), init_pos_.y());
            param_.wp_strike += Eigen::Vector2d(init_pos_.x(), init_pos_.y());

            current_state_ = MissionState::TAKEOFF;
            ROS_INFO(">>> [任务开始] 起飞点已锁定，开始爬升！");
        }
        break;

    case MissionState::TAKEOFF:
        publishSetpoint(Eigen::Vector2d(init_pos_.x(), init_pos_.y()), Eigen::Vector2d(0, 0), init_pos_.z() + param_.takeoff_height, init_yaw_);
        if (std::abs(current_pos_.z() - param_.takeoff_height) < 0.15)
        {

            // [修复] 不立刻寻路！切入等图状态
            current_state_ = MissionState::WAIT_FOR_MAP;
            map_wait_start_time_ = ros::Time::now();

            nh_.setParam("/pcl_enable", true);
            ROS_INFO("[Boss] PCL 已启用！原位悬停等待点云积累...");
            planner_manager_->buildWalls(init_pos_.x(), init_pos_.y());
        }
        break;

    // ==========================================================
    // [史诗级修复] 时序对齐锁 (Wait for Map)
    // 强行原地悬停 2.0 秒。让 PCL 雷达子弹飞一会，把障碍物填满地图。
    // 彻底杜绝 A* 趁着地图是空的，画出一条直线直接撞墙的 Bug！
    // ==========================================================
    case MissionState::WAIT_FOR_MAP:
        publishSetpoint(Eigen::Vector2d(init_pos_.x(), init_pos_.y()), Eigen::Vector2d(0, 0), init_pos_.z() + param_.takeoff_height, init_yaw_);

        if ((ros::Time::now() - map_wait_start_time_).toSec() > 2.0)
        {
            current_state_ = MissionState::NAV_RECOG_AREA;
            has_global_plan_ = false;
            ROS_INFO(">>> [任务一] 地图载入完毕，前往目标识别区！");
        }
        break;

    case MissionState::NAV_RECOG_AREA:
        if (flyToXY(param_.wp_recog))
        {
            current_state_ = MissionState::HOVER_RECOGNIZE;
            hover_start_time_ = ros::Time::now();
            ROS_INFO(">>>[任务二] 到达识别区，开始 3 秒悬停...");
        }
        break;

    case MissionState::HOVER_RECOGNIZE:
        publishSetpoint(param_.wp_recog, Eigen::Vector2d(0, 0), init_pos_.z() + param_.takeoff_height, init_yaw_);
        if ((ros::Time::now() - hover_start_time_).toSec() > 3.0)
        {
            current_state_ = MissionState::NAV_AIRDROP_AREA;
            has_global_plan_ = false;
            ROS_INFO(">>> [任务三] 识别完毕！前往投放区...");
        }
        break;

    case MissionState::NAV_AIRDROP_AREA:
        if (flyToXY(param_.wp_airdrop))
        {
            current_state_ = MissionState::HOVER_AIRDROP;
            hover_start_time_ = ros::Time::now();
            ROS_INFO(">>> [任务四] 到达投放区上方，悬停准备...");
        }
        break;

    case MissionState::HOVER_AIRDROP:
        publishSetpoint(param_.wp_airdrop, Eigen::Vector2d(0, 0), init_pos_.z() + param_.takeoff_height, init_yaw_);
        if ((ros::Time::now() - hover_start_time_).toSec() > 3.0)
        {
            current_state_ = MissionState::NAV_STRIKE_AREA;
            has_global_plan_ = false;
            ROS_INFO(">>> [任务五] 投掷完毕！前往靶标攻击方位...");
        }
        break;

    case MissionState::NAV_STRIKE_AREA:
        if (flyToXY(param_.wp_strike))
        {
            current_state_ = MissionState::LASER_STRIKE;
            hover_start_time_ = ros::Time::now();
            ROS_INFO(">>> [任务五] 抵达攻击阵位，准备激光打击！");
        }
        break;

    case MissionState::LASER_STRIKE:
        publishSetpoint(param_.wp_strike, Eigen::Vector2d(0, 0), init_pos_.z() + param_.takeoff_height, init_yaw_);
        if ((ros::Time::now() - hover_start_time_).toSec() > 2.0)
        {
            current_state_ = MissionState::RETURN_TO_LAUNCH;
            has_global_plan_ = false;
            ROS_INFO(">>> [任务六] 攻击成功！全速返航！");
        }
        break;

    case MissionState::RETURN_TO_LAUNCH:
        if (flyToXY(Eigen::Vector2d(init_pos_.x(), init_pos_.y())))
        {
            current_state_ = MissionState::LANDING;
            ROS_INFO(">>> [任务六] 回到起飞点上空，开始降落...");
        }
        break;

    case MissionState::LANDING:
        publishSetpoint(Eigen::Vector2d(init_pos_.x(), init_pos_.y()), Eigen::Vector2d(0, 0), init_pos_.z() + current_pos_.z() - 0.2, init_yaw_);
        if (current_pos_.z() < init_pos_.z() + 0.1)
        {
            current_state_ = MissionState::FINISHED;
            ROS_INFO(">>> 比赛完美收官！停桨！");
        }
        break;

    case MissionState::FINISHED:
        publishSetpoint(Eigen::Vector2d(init_pos_.x(), init_pos_.y()), Eigen::Vector2d(0, 0), init_pos_.z() + 0.0, init_yaw_);
        break;
    }
    // ==========================================================
    // [新增] 5Hz 频率刷新 RViz，让你一眼看穿所有 Bug！
    // ==========================================================
    static int viz_count = 0;
    if (++viz_count % 4 == 0)
    {
        if (planner_manager_)
        {
            planner_manager_->publishVisualization();
        }
    }
}

// [史诗级修复] 引入前馈速度控制，并修正高度坐标基准
void MissionController::publishSetpoint(const Eigen::Vector2d &xy, const Eigen::Vector2d &vel_xy, double z, double yaw)
{
    mavros_msgs::PositionTarget msg;
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;


    msg.type_mask = 3040;

    msg.position.x = xy.x();
    msg.position.y = xy.y();
    msg.position.z = z;

    msg.velocity.x = vel_xy.x();
    msg.velocity.y = vel_xy.y();
    msg.velocity.z = 0;

    msg.yaw = yaw;
    setpoint_pub_.publish(msg);
}


bool MissionController::flyToXY(const Eigen::Vector2d &target_xy)
{
    Eigen::Vector2d curr_xy(current_pos_.x(), current_pos_.y());
    double dist_to_goal = (curr_xy - target_xy).norm();
    double absolute_target_z = init_pos_.z() + param_.takeoff_height;

    if (dist_to_goal < 0.2)
        return true;

    // [新增] 用于锁死悬停点的静态变量
    static Eigen::Vector2d hover_pt = curr_xy;
    // 监控输出
    ROS_INFO_THROTTLE(1.0, "[Boss] 巡航中 -> 距终点: %.2fm, 真实Z: %.2f，目前锚点: (%.2f, %.2f)", dist_to_goal, current_pos_.z(), hover_pt.x(), hover_pt.y());

    bool need_replan = !has_global_plan_ || planner_manager_->checkCollision();
    if (need_replan)
    {
        if (!is_planning_.load())
        {
            is_planning_.store(true);
            has_global_plan_ = false;
            // 【核心修复】在丢失轨迹的那一瞬间，死死锁住当前坐标作为保命锚点！
            hover_pt = curr_xy;

            ROS_WARN_THROTTLE(1.0, "[Boss] 🚨 触发重规划！启动后台子线程计算...");

            Eigen::Vector2d start_pt = curr_xy;
            Eigen::Vector2d end_pt = target_xy;

            std::thread([this, start_pt, end_pt]()
                        {
                            ROS_INFO("[子线程] 🌐 线程 %zu 开始寻路计算...", std::hash<std::thread::id>{}(std::this_thread::get_id()));
                bool success = planner_manager_->replan(start_pt, end_pt);
                has_global_plan_ = success;
                is_planning_.store(false); })
                .detach();
        }
        // =========================================================================
        // [史诗级保命防线] Fallback 机制！
        // =========================================================================
        // 防线一：旧轨迹复用！即使需要 replan，先看看旧轨迹接下来 1.5 秒还能不能飞？
        if (has_global_plan_ && !planner_manager_->checkCollisionLocal(1.5))
        {
            ROS_WARN_THROTTLE(1.0, "[Boss] 🛡️ 规划失败或延迟，但历史轨迹安全！继续沿旧轨迹盲飞！");
            double t_sec = (ros::Time::now() - planner_manager_->getTrajStartTime()).toSec();
            Eigen::Vector2d cmd_pos = planner_manager_->getPosition(t_sec);
            Eigen::Vector2d cmd_vel = planner_manager_->getVelocity(t_sec);
            publishSetpoint(cmd_pos, cmd_vel, absolute_target_z, init_yaw_);
            return false;
        }

        // 防线二：柔性刹车！如果旧轨迹也撞墙了，绝不原地急停，顺着速度向量缓冲 0.8 秒！
        ROS_WARN_THROTTLE(0.5, "[Boss] 🛑 失去所有安全轨迹！执行柔性刹车紧急避险！");
        has_global_plan_ = false;                             // 彻底废弃旧轨迹
        Eigen::Vector2d brake_pos = curr_xy + curr_vel * 0.8; // 顺着惯性往前滑行 0.8 秒的距离
        publishSetpoint(brake_pos, Eigen::Vector2d(0, 0), absolute_target_z, init_yaw_);
        return false;
    }

    // 正常循迹：同时索要【位置】和【速度】
    double t_sec = (ros::Time::now() - planner_manager_->getTrajStartTime()).toSec();
    Eigen::Vector2d cmd_pos = planner_manager_->getPosition(t_sec);
    Eigen::Vector2d cmd_vel = planner_manager_->getVelocity(t_sec);

    // 发送带有前馈速度的完美控制指令！
    publishSetpoint(cmd_pos, cmd_vel,absolute_target_z, init_yaw_);

    return false;
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
        publishSetpoint(Eigen::Vector2d(current_pos_.x(), current_pos_.y()),Eigen::Vector2d(0, 0),
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
    // [新增] 记录当前线速度，用于柔性刹车
    current_vel_.x() = msg->twist.twist.linear.x;
    current_vel_.y() = msg->twist.twist.linear.y;
    current_vel_.z() = msg->twist.twist.linear.z;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    double r, p;
    tf::Matrix3x3(q).getRPY(r, p, current_yaw_);
}