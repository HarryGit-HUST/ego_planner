#include"bspline_optimizer.h"

BsplineOptimizer::BsplineOptimizer()
{
}
BsplineOptimizer::~BsplineOptimizer()
{
}
void BsplineOptimizer::setParam(ros::NodeHandle &nh)
{
    // 从 ROS 参数服务器读取权重和物理极限
    nh.param("bspline_optimizer/weight_smooth", param_.weight_smooth, 1.0);
    nh.param("bspline_optimizer/weight_collision", param_.weight_collision, 1.0);
    nh.param("bspline_optimizer/weight_feasibility", param_.weight_feasibility, 1.0);
    nh.param("bspline_optimizer/max_vel", param_.max_vel, 1.0);
    nh.param("bspline_optimizer/max_acc", param_.max_acc, 1.0);
    nh.param("bspline_optimizer/safe_distance", param_.safe_distance, 0.5);
    nh.param("bspline_optimizer/max_iteration_num", param_.max_iteration_num, 300);
}
void BsplineOptimizer::setEnvironment(std::shared_ptr<GridMap> map)
{
    grid_map_ = map;
}
static double BsplineOptimizer::costFunction(void *instance, const double *x, double *grad, const int n)
{
    // 魔法：把 void* 变回你的类指针！
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(instance);
    double cost;
    // 将一维数组 x 映射成 Eigen 矩阵 (无需拷贝内存！)
    Eigen::Map<const Eigen::MatrixXd> q(x, 2, n / 2);
    Eigen::Map<Eigen::MatrixXd> g(grad, 2, n / 2);
    opt->combineCost(q, cost, g);
    return cost;
}
void BsplineOptimizer::combineCost(const double *x, double *grad, double &f_combine)
{
    f_combine = 0.0;
    Eigen::Map<const Eigen::MatrixXd> q(x, 2, pt_num_); // 当前控制点矩阵
    Eigen::Map<Eigen::MatrixXd> g(grad, 2, pt_num_); // 当前梯度矩阵
    Eigen::MatrixXd grad_smooth(q.rows(), q.cols());
    Eigen::MatrixXd grad_collision(q.rows(), q.cols());
    Eigen::MatrixXd grad_feasibility(q.rows(), q.cols());
    double cost_smooth, cost_collision, cost_feasibility;
    // 计算各项代价和梯度
    calcSmoothnessCost(q, cost_smooth, grad_smooth);
    calcCollisionCost(q, cost_collision, grad_collision);
    calcFeasibilityCost(q, cost_feasibility, grad_feasibility);
    // 汇总总代价
    f_combine = cost_smooth + cost_collision + cost_feasibility;
    // 汇总总梯度
    g = grad_smooth + grad_collision + grad_feasibility;


}
void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
{
    cost = 0.0;
    gradient.setZero(); // 初始化梯度矩阵
}
void BsplineOptimizer::calcCollisionCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
{
    cost = 0.0;
    gradient.setZero(); // 初始化梯度矩阵

    Eigen::Vector3d grad_dir; // 障碍物梯度方向（指向自由空间）
    double depth;             
    // TODO: 遍历控制点 q.col(i)
    for(int i = 0; i < pt_num_; ++i)
    {
        Eigen::Vector3d pt = q.col(i); // 当前控制点位置 (x, y, z)

        // TODO: 调用 grid_map_->getObstacleGradient(q.col(i), grad_dir, depth)
        if (grid_map_->getObstacleGradient(pt, grad_dir, depth))
        {
            // TODO: 如果撞了，累加 cost，并把 grad_dir * weight 累加到 gradient.col(i) 里
            if(depth < param_.safe_distance)
            {
                double cost_increment = param_.weight_collision * (param_.safe_distance - depth);
                cost += cost_increment;

                // 梯度方向乘以权重，累加到对应控制点的梯度上
                gradient.col(i) += param_.weight_collision * grad_dir;
            }
        }

        
    }
}
bool BsplineOptimizer::optimize(Eigen::MatrixXd &ctrl_pts, double ts)
{
    // 1. 保存优化参数
    pt_num_ = ctrl_pts.cols();
    bspline_interval_ = ts;

    // 2. 将 2D 控制点矩阵展平为一维数组（供 C 库使用）
    // 格式: [x0, y0, x1, y1, ..., xn, yn]
    std::vector<double> x_vec(ctrl_pts.size());
    Eigen::Map<Eigen::VectorXd>(x_vec.data(), ctrl_pts.size()) =
        Eigen::Map<const Eigen::VectorXd>(ctrl_pts.data(), ctrl_pts.size());

    // 3. L-BFGS 参数配置
    lbfgs_parameter_t lbfgs_params;
    lbfgs_parameter_init(&lbfgs_params);
    lbfgs_params.max_iterations = param_.max_iteration_num;
    lbfgs_params.epsilon = 1e-5; // 收敛阈值
    lbfgs_params.past = 3;       // 检查最近3次迭代的进步

    // 4. 调用 L-BFGS 优化
    double final_cost;
    int ret = lbfgs_optimize(
        x_vec.size(),                   // 变量总数
        x_vec.data(),                   // 初始值 & 输出结果
        &final_cost,                    // 最终代价
        BsplineOptimizer::costFunction, // 回调函数（静态）
        nullptr,                        // 可选：进度回调
        this,                           // instance 指针，传给 costFunction
        &lbfgs_params                   // 参数配置
    );

    // 5. 将优化结果写回 ctrl_pts
    if (ret >= 0) // ret >= 0 表示成功收敛
    {
        ctrl_pts = Eigen::Map<const Eigen::MatrixXd>(x_vec.data(), 2, pt_num_);
        ROS_INFO("[Optimizer] Success! Cost: %.4f, Iter: %d", final_cost, ret);
        return true;
    }
    else
    {
        ROS_WARN("[Optimizer] Failed! Error code: %d", ret);
        return false;
    }
}