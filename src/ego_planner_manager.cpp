#include "ego_planner_manager.h"
#include <iostream>
// [修复 9]：极其关键！既然你在这个 cpp 里用到了这些类的具体方法，必须包含它们的真实头文件！
#include "grid_map.h"
#include "a_star.h"
#include "bspline_optimizer.h"
PlannerManager::PlannerManager() {}
PlannerManager::~PlannerManager() {}
// (初始化等基础函数省略，你需要自己把指针实例化、并赋值给下属)
void PlannerManager::init(ros::NodeHandle &nh)
{
    // 1. 从ROS参数服务器读取配置
    nh.param("manager/max_vel", param_.max_vel, 1.0);
    nh.param("manager/max_acc", param_.max_acc, 1.0);
    nh.param("manager/ctrl_pt_dist", param_.ctrl_pt_dist, 0.1);

    // 2. 实例化所有子模块（关键：智能指针初始化）
    grid_map_ = std::make_shared<GridMap>();
    a_star_ = std::make_shared<AStar>();
    optimizer_ = std::make_shared<BsplineOptimizer>();

    // 3. 子模块初始化（传递参数、地图指针）
    grid_map_->init(nh);
    a_star_->init(grid_map_); // [修复] 名字统一为 init

    optimizer_->setEnvironment(grid_map_); // [修复] 名字统一
    optimizer_->setParam(nh);

    // 1. 在 init 函数末尾加上：
    astar_pub_ = nh.advertise<nav_msgs::Path>("/viz/astar_path", 1);
    bspline_pub_ = nh.advertise<visualization_msgs::Marker>("/viz/bspline_traj", 1);
    ROS_INFO("[CEO] 规划总局初始化完成.");
}

bool PlannerManager::replan(const Eigen::Vector2d &start_pt, const Eigen::Vector2d &target_pt)
{
    ROS_INFO("[CEO] 收到规划指令: 起点(%.2f, %.2f) -> 终点(%.2f, %.2f)",
             start_pt.x(), start_pt.y(), target_pt.x(), target_pt.y());

    std::vector<Eigen::Vector2d> astar_path;
    if (!a_star_->search(start_pt, target_pt, astar_path))
    {
        ROS_WARN("[CEO] A* 寻路失败！无法到达终点！");
        return false;
    }

    last_astar_path_ = astar_path;
    ROS_INFO("[CEO] A* 寻路成功，拐角节点数: %zu", astar_path.size());

    // =========================================================================
    // [史诗级修复 1] 路径重采样 (Resampling)：将稀疏的拐角点变成密集的控制点！
    // =========================================================================
    std::vector<Eigen::Vector2d> dense_path;
    for (size_t i = 0; i < astar_path.size() - 1; ++i)
    {
        Eigen::Vector2d p1 = astar_path[i];
        Eigen::Vector2d p2 = astar_path[i + 1];
        double seg_len = (p2 - p1).norm();

        // 按照 yaml 里的 ctrl_pt_dist (比如 0.3m) 插入中间点
        int num_pts = std::max(1, (int)std::ceil(seg_len / param_.ctrl_pt_dist));
        for (int j = 0; j < num_pts; ++j)
        {
            dense_path.push_back(p1 + (p2 - p1) * ((double)j / num_pts));
        }
    }
    dense_path.push_back(astar_path.back()); // 补上最后一个点

    // [史诗级修复 2] 正确的时间步长 ts
    // 现在相邻两个点之间的物理距离严格等于 ctrl_pt_dist
    // 匀速飞行的时间间隔 ts = 距离 / 最大速度
    double ts = param_.ctrl_pt_dist / param_.max_vel;
    if (ts < 0.05)
        ts = 0.05; // 防 NaN 保护

    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, dense_path, std::vector<Eigen::Vector2d>(), ctrl_pts);

    ROS_INFO("[CEO] 重采样完成，生成密集控制点数: %ld, ts: %.3f", ctrl_pts.cols(), ts);

    // 呼叫 L-BFGS！此时有几十个自由点，一旦碰到障碍物，优化器会疯狂迭代将其推开！
    if (!optimizer_->optimize(ctrl_pts, ts))
    {
        ROS_ERROR("[CEO] B样条轨迹优化崩溃！");
        return false;
    }

    local_traj_.setUniformBspline(ctrl_pts, 3, ts);

    // [核心修复] 设置动力学极限，否则 checkFeasibility 会除以零导致 NaN！
    local_traj_.setPhysicalLimits(param_.max_vel, param_.max_acc);

    double ratio;
    if (!local_traj_.checkFeasibility(ratio, false))
    {
        // [保命断路器] 如果优化出来的轨迹极其扭曲，导致需要把时间拉长 3 倍以上
        // 说明这是一条“垃圾轨迹”，直接丢弃，让飞控走 Fallback 防线！
        if (ratio > 10.0)
        {
            ROS_ERROR("[CEO] 优化轨迹极其扭曲 (需减速 %.1f 倍)，主动废弃该轨迹！", ratio);
            return false;
        }
        ROS_WARN("[CEO] 轨迹超速，正在进行动力学时间拉长 %.2f 倍...", ratio);
        ratio = std::min(ratio, 5.0); // 防止过度拉长导致飞行时间过长
        local_traj_.lengthenTime(ratio);
    }

    traj_start_time_ = ros::Time::now();
    ROS_INFO("[CEO] ✅ 全局轨迹生成成功并归档！总飞行时长: %.2f 秒", local_traj_.getTimeSum());
    return true;
}

Eigen::Vector2d PlannerManager::getPosition(double t_sec)
{
    // [防越界保护]
    if (local_traj_.getControlPoints().cols() == 0)
        return Eigen::Vector2d(0, 0);

    double t_sum = local_traj_.getTimeSum();
    if (t_sec > t_sum)
        return local_traj_.evaluateDeBoor(t_sum);
    else
        return local_traj_.evaluateDeBoor(t_sec);
}
Eigen::Vector2d PlannerManager::getVelocity(double t_sec)
{
    if (local_traj_.getControlPoints().cols() == 0)
        return Eigen::Vector2d(0, 0);
    double t_sum = local_traj_.getTimeSum();
    if (t_sec > t_sum)
        return Eigen::Vector2d(0, 0);
    else
    {
        UniformBspline derivative_traj = local_traj_.getDerivative();
        return derivative_traj.evaluateDeBoor(t_sec);
    }
}
bool PlannerManager::checkCollision()
{
    if (local_traj_.getControlPoints().cols() == 0)
        return false;
    double duration = local_traj_.getTimeSum();

    // [核心逻辑修复]：跳过前 0.5 秒的碰撞检测！
    // 只要我接下来的 0.5 秒不会真撞上死墙，就允许我擦着膨胀区飞出去！
    for (double t = 0.5; t <= duration; t += 0.1)
    {
        Eigen::Vector2d pt = local_traj_.evaluateDeBoor(t);
        if (grid_map_->isOccupied(pt))
        {
            ROS_WARN_THROTTLE(1.0, "[CEO 警报] 轨迹未来 %.1fs 处受阻！触发重规划！", t);
            return true;
        }
    }
    return false;
}

void PlannerManager::buildWalls(double start_x, double start_y)
{
    // CEO 不亲自干活，直接使唤底层的 grid_map_ 部门去建墙
    if (grid_map_ != nullptr)
    {
        grid_map_->buildStaticWalls(start_x, start_y);
    }
}

void PlannerManager::publishVisualization()
{
    if (grid_map_)
        grid_map_->publishMap();

    // 发布 A* 蓝线
    if (!last_astar_path_.empty())
    {
        nav_msgs::Path msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        for (const auto &p : last_astar_path_)
        {
            geometry_msgs::PoseStamped ps;
            ps.pose.position.x = p.x();
            ps.pose.position.y = p.y();
            ps.pose.position.z = 1.0;
            ps.pose.orientation.w = 1.0;
            msg.poses.push_back(ps);
        }
        astar_pub_.publish(msg);
    }

    // 发布 B 样条绿球
    if (local_traj_.getControlPoints().cols() > 0 && local_traj_.getTimeSum() > 0.0)
    {
        visualization_msgs::Marker msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        msg.ns = "bspline";
        msg.id = 0;
        msg.type = visualization_msgs::Marker::SPHERE_LIST;
        msg.action = visualization_msgs::Marker::ADD;
        msg.scale.x = 0.15;
        msg.scale.y = 0.15;
        msg.scale.z = 0.15;
        msg.color.a = 1.0;
        msg.color.r = 0.0;
        msg.color.g = 1.0;
        msg.color.b = 0.0;

        double t_sum = local_traj_.getTimeSum();
        for (double t = 0; t <= t_sum; t += 0.05)
        {
            Eigen::Vector2d pt = local_traj_.evaluateDeBoor(t);
            geometry_msgs::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = 1.0;
            msg.points.push_back(p);
        }
        bspline_pub_.publish(msg);
    }
}

bool PlannerManager::checkCollisionLocal(double time_horizon)
{
    if (local_traj_.getControlPoints().cols() == 0)
        return true; // 没轨迹当然不安全

    double t_sec = (ros::Time::now() - traj_start_time_).toSec();
    double t_end = std::min(t_sec + time_horizon, local_traj_.getTimeSum());

    // 沿着接下来的 time_horizon 时间段，每隔 0.1 秒检查一次前方会不会撞
    for (double t = t_sec; t <= t_end; t += 0.1)
    {
        Eigen::Vector2d pt = local_traj_.evaluateDeBoor(t);
        if (grid_map_->isOccupied(pt))
            return true; // 会撞！
    }
    return false; // 安全！
}