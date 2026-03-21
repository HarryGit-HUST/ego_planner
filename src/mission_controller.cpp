#include "mission_controller.h"
#include "ego_planner_manager.h"
#include <cmath>
#include <functional>
#include <tf/transform_datatypes.h>

MissionController::MissionController()
    : current_state_(MissionState::IDLE),
      current_pos_(Eigen::Vector3d::Zero()),
      current_vel_(Eigen::Vector3d::Zero()),
      current_yaw_(0.0),
      is_connected_(false),
      has_global_plan_(false),
      init_yaw_(0.0),
      is_planning_(false) {}

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
    double absolute_target_z = init_pos_.z() + param_.takeoff_height;
    Eigen::Vector2d curr_xy(current_pos_.x(), current_pos_.y());

    // 检查是否到达目标点（允许 0.15m 误差）
    if ((curr_xy - target_xy).norm() < 0.15)
    {
        return true;
    }

    // [修复] 只在真正必要时才触发重规划，增加冷却时间防止频繁切换
    static ros::Time last_replan_time = ros::Time(0);
    double time_since_last_replan = (ros::Time::now() - last_replan_time).toSec();

    bool collision_detected = planner_manager_->checkCollision();
    bool need_replan = !has_global_plan_ || (collision_detected && time_since_last_replan > 1.0);

    if (need_replan)
    {
        if (!is_planning_.load())
        {
            is_planning_.store(true);

            // 【核心修复】新轨迹起点 = 当前位置 + 当前速度方向的前馈
            // 问题根源：之前用旧轨迹预测点作为新起点，但无人机可能已经偏离旧轨迹
            // 解决方案：直接用当前位置作为新起点，保证轨迹连续性
            Eigen::Vector2d start_pt;
            
            if (has_global_plan_)
            {
                // 计算当前时刻在旧轨迹上的理论位置
                double t_now = (ros::Time::now() - planner_manager_->getTrajStartTime()).toSec();
                Eigen::Vector2d traj_pos = planner_manager_->getPosition(t_now);
                Eigen::Vector2d traj_vel = planner_manager_->getVelocity(t_now);
                
                // 计算当前位置与轨迹位置的偏差
                double pos_error = (curr_xy - traj_pos).norm();
                
                ROS_INFO_THROTTLE(1.0, "[Boss] 🔄 轨迹重规划 | 当前位置 (%.2f, %.2f) | 轨迹位置 (%.2f, %.2f) | 偏差 %.3fm",
                                  curr_xy.x(), curr_xy.y(), traj_pos.x(), traj_pos.y(), pos_error);
                
                // [关键决策] 如果偏差超过 0.3m，说明无人机已经偏离旧轨迹
                // 此时应该用当前位置作为新起点，而不是预测点
                if (pos_error > 0.3)
                {
                    // 使用当前位置 + 速度方向的前馈补偿（0.2 秒预测）
                    start_pt = curr_xy + current_vel_.head<2>() * 0.2;
                    ROS_INFO_THROTTLE(1.0, "[Boss] ⚠️  位置偏差过大！使用当前位置 + 速度前馈作为新起点");
                }
                else
                {
                    // 偏差较小，使用轨迹上的预测点（0.2 秒后）
                    double t_future = t_now + 0.2;
                    start_pt = planner_manager_->getPosition(t_future);
                    ROS_INFO_THROTTLE(1.0, "[Boss] ✓ 位置偏差正常，使用轨迹预测点作为新起点");
                }
            }
            else
            {
                // 彻底没轨迹了，从当前位置起步
                start_pt = curr_xy;
            }

            std::thread([this, start_pt, target_xy]()
                        {
                    bool success = planner_manager_->replan(start_pt, target_xy);
                    // 只有成功了才覆盖全局状态
                    if(success) {
                        has_global_plan_ = true;
                    }
                    is_planning_.store(false); })
                .detach();

            // [修复] 在主线程记录重规划时间，避免 lambda 捕获静态变量
            last_replan_time = ros::Time::now();
        }

        // 【核心控制流】主线程绝对不阻塞！
        if (has_global_plan_)
        {
            // 在子线程计算期间，继续顺着旧轨迹飞！绝不原地急停！
            double t_sec = (ros::Time::now() - planner_manager_->getTrajStartTime()).toSec();
            Eigen::Vector2d cmd_pos = planner_manager_->getPosition(t_sec);
            Eigen::Vector2d cmd_vel = planner_manager_->getVelocity(t_sec);
            publishSetpoint(cmd_pos, cmd_vel, absolute_target_z, init_yaw_);
            return false;
        }
        else
        {
            // 连旧轨迹都没了，只能执行柔性刹车
            ROS_WARN_THROTTLE(0.5, "[Boss] 🛑 无安全轨迹可用！执行柔性刹车紧急避险！");
            Eigen::Vector2d curr_vel = current_vel_.head<2>();
            Eigen::Vector2d brake_pos = curr_xy + curr_vel * 0.8;
            publishSetpoint(brake_pos, Eigen::Vector2d(0, 0), absolute_target_z, init_yaw_);
            return false;
        }
    }

    // 有有效轨迹时，沿轨迹飞行
    if (has_global_plan_)
    {
        double t_sec = (ros::Time::now() - planner_manager_->getTrajStartTime()).toSec();

        // [关键修复] 限制 t_sec 不超过轨迹总时长，防止越界
        double traj_duration = planner_manager_->getTrajDuration();
        if (t_sec > traj_duration)
        {
            t_sec = traj_duration;
        }

        Eigen::Vector2d cmd_pos = planner_manager_->getPosition(t_sec);
        Eigen::Vector2d cmd_vel = planner_manager_->getVelocity(t_sec);
        publishSetpoint(cmd_pos, cmd_vel, absolute_target_z, init_yaw_);
    }

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