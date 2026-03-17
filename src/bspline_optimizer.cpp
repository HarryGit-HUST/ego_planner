#include "bspline_optimizer.h"
#include <iostream>
#include "grid_map.h"

BsplineOptimizer::BsplineOptimizer() {}
BsplineOptimizer::~BsplineOptimizer() {}

void BsplineOptimizer::setParam(ros::NodeHandle &nh)
{
    nh.param("bspline_optimizer/weight_smooth", param_.weight_smooth, 1.0);
    nh.param("bspline_optimizer/weight_collision", param_.weight_collision, 2.0); // 避障权重大一点
    nh.param("bspline_optimizer/weight_feasibility", param_.weight_feasibility, 0.1);
    nh.param("bspline_optimizer/max_vel", param_.max_vel, 1.0);
    nh.param("bspline_optimizer/max_acc", param_.max_acc, 1.0);
    nh.param("bspline_optimizer/safe_distance", param_.safe_distance, 0.5);
    nh.param("bspline_optimizer/max_iteration_num", param_.max_iteration_num, 300);
}

void BsplineOptimizer::setEnvironment(std::shared_ptr<GridMap> map)
{
    grid_map_ = map;
}

// ============================================================================
// [核心修复] L-BFGS 回调接口：只处理“自由控制点”
// ============================================================================
double BsplineOptimizer::costFunction(void *instance, const double *x, double *grad, const int n)
{
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(instance);
    double cost;

    // n 是自由变量的总维度 (自由点数 * 2)。我们将一维 x 映射为 Eigen 矩阵
    Eigen::Map<const Eigen::MatrixXd> q_free(x, 2, n / 2);
    Eigen::Map<Eigen::MatrixXd> g_free(grad, 2, n / 2);

    opt->combineCost(q_free, cost, g_free);
    return cost;
}

void BsplineOptimizer::combineCost(const Eigen::MatrixXd &q_free, double &f_combine, Eigen::MatrixXd &g_free)
{
    // 1. 将固定的端点和自由点拼接成【完整的控制点矩阵 q】
    Eigen::MatrixXd q = control_points_; // 拷贝一份带端点的初始矩阵
    // 阶数 p=3，前 3 个和后 3 个点锁死不动
    q.block(0, 3, 2, q_free.cols()) = q_free;

    // 2. 初始化完整的梯度矩阵
    Eigen::MatrixXd gradient = Eigen::MatrixXd::Zero(2, pt_num_);

    double cost_smooth = 0, cost_collision = 0, cost_feasibility = 0;
    Eigen::MatrixXd grad_smooth = Eigen::MatrixXd::Zero(2, pt_num_);
    Eigen::MatrixXd grad_collision = Eigen::MatrixXd::Zero(2, pt_num_);
    Eigen::MatrixXd grad_feasibility = Eigen::MatrixXd::Zero(2, pt_num_);

    // 3. 计算各项代价和梯度 (你的数学推导非常完美！)
    calcSmoothnessCost(q, cost_smooth, grad_smooth);
    calcCollisionCost(q, cost_collision, grad_collision);
    calcFeasibilityCost(q, cost_feasibility, grad_feasibility);

    f_combine = cost_smooth + cost_collision + cost_feasibility;
    gradient = grad_smooth + grad_collision + grad_feasibility;

    // 4. [核心修复] 过滤掉首尾固定的点，只把中间自由点的梯度传回给 L-BFGS
    g_free = gradient.block(0, 3, 2, q_free.cols());
}

bool BsplineOptimizer::optimize(Eigen::MatrixXd &ctrl_pts, double ts)
{
    pt_num_ = ctrl_pts.cols();
    bspline_interval_ = ts;
    control_points_ = ctrl_pts; // 备份一份，保留固定端点

    // [核心修复] 如果路径太短，自由点数量 <= 0，直接跳过优化！
    int free_num = pt_num_ - 6;
    if (free_num <= 0)
        return true;

    // 取出中间的自由点展平送给 L-BFGS
    Eigen::MatrixXd q_free = ctrl_pts.block(0, 3, 2, free_num);
    std::vector<double> x_vec(q_free.size());
    Eigen::Map<Eigen::VectorXd>(x_vec.data(), q_free.size()) =
        Eigen::Map<const Eigen::VectorXd>(q_free.data(), q_free.size());

    lbfgs_parameter_t lbfgs_params;
    lbfgs_parameter_init(&lbfgs_params);
    lbfgs_params.max_iterations = param_.max_iteration_num;
    lbfgs_params.epsilon = 1e-4; // 稍微放宽，加快收敛
    lbfgs_params.past = 3;

    double final_cost;
    int ret = lbfgs_optimize(
        x_vec.size(), x_vec.data(), &final_cost,
        BsplineOptimizer::costFunction, nullptr, this, &lbfgs_params);

    if (ret >= 0 || ret == LBFGS_ALREADY_MINIMIZED)
    {
        // 将优化后的自由点写回
        Eigen::MatrixXd q_free_opt = Eigen::Map<const Eigen::MatrixXd>(x_vec.data(), 2, free_num);
        ctrl_pts.block(0, 3, 2, free_num) = q_free_opt;
        return true;
    }
    else
    {
        ROS_WARN("[Optimizer] L-BFGS 失败! 错误码: %d", ret);
        return false;
    }
}

// 你的数学公式完美保留，仅增加边界保护
void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
{
    for (int i = 1; i < pt_num_ - 1; i++)
    { // [修复] 循环边界修正为 -1
        Eigen::Vector2d V = q.col(i + 1) - 2 * q.col(i) + q.col(i - 1);
        cost += V.squaredNorm() * param_.weight_smooth;
        gradient.col(i - 1) += 2.0 * param_.weight_smooth * V;
        gradient.col(i) += -4.0 * param_.weight_smooth * V;
        gradient.col(i + 1) += 2.0 * param_.weight_smooth * V;
    }
}

void BsplineOptimizer::calcCollisionCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
{
    Eigen::Vector2d grad_dir;
    double dist;
    for (int i = 0; i < pt_num_; ++i)
    {
        if (grid_map_->getObstacleGradient(q.col(i), grad_dir, dist))
        {
            if (dist < param_.safe_distance)
            {
                double penalty = param_.safe_distance - dist;
                cost += param_.weight_collision * penalty * penalty;
                gradient.col(i) += -2.0 * param_.weight_collision * penalty * grad_dir;
            }
        }
    }
}

void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
{
    for (int i = 0; i < pt_num_ - 1; i++)
    {
        Eigen::Vector2d V = (q.col(i + 1) - q.col(i)) / bspline_interval_;
        double speed_sq = V.squaredNorm();
        double max_speed_sq = param_.max_vel * param_.max_vel;

        if (speed_sq > max_speed_sq)
        {
            double penalty = speed_sq - max_speed_sq;
            cost += param_.weight_feasibility * penalty * penalty;
            Eigen::Vector2d grad_V = 4.0 * param_.weight_feasibility * penalty * V;
            gradient.col(i) += -grad_V / bspline_interval_;
            gradient.col(i + 1) += grad_V / bspline_interval_;
        }
    }
}