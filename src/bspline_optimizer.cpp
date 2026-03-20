#include "bspline_optimizer.h"
#include <iostream>
#include "grid_map.h"

BsplineOptimizer::BsplineOptimizer() {}
BsplineOptimizer::~BsplineOptimizer() {}

void BsplineOptimizer::setParam(ros::NodeHandle &nh)
{
    nh.param("bspline_optimizer/weight_smooth", param_.weight_smooth, 1.0);
    nh.param("bspline_optimizer/weight_collision", param_.weight_collision, 2.0); 
    nh.param("bspline_optimizer/weight_feasibility", param_.weight_feasibility, 0.1);
    nh.param("bspline_optimizer/max_vel", param_.max_vel, 1.0);
    nh.param("bspline_optimizer/max_acc", param_.max_acc, 1.0);
    nh.param("bspline_optimizer/safe_distance", param_.safe_distance, 0.5);
    nh.param("bspline_optimizer/max_iteration_num", param_.max_iteration_num, 300);
    ROS_INFO("[Optimizer] 优化器参数加载完成.");
}

void BsplineOptimizer::setEnvironment(std::shared_ptr<GridMap> map)
{
    grid_map_ = map;
}

// ============================================================================
//[魔法回调] L-BFGS 计算代价与梯度
// ============================================================================
double BsplineOptimizer::costFunction(void *instance, const double *x, double *grad, const int n, const double step)
{
    BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(instance);
    double cost = 0;

    // 拷贝内存给 Eigen 计算
    Eigen::MatrixXd q_free = Eigen::Map<const Eigen::MatrixXd>(x, 2, n / 2);
    Eigen::MatrixXd g_free = Eigen::MatrixXd::Zero(2, n / 2);

    opt->combineCost(q_free, cost, g_free);

    // [防御性编程] 检查梯度是否出现 NaN 或 Inf，如果出现，强行清零防止崩溃！
    if (!g_free.allFinite())
    {
        ROS_ERROR_THROTTLE(1.0, "[Optimizer] 🚨 警告：计算出无效梯度 (NaN/Inf)，已强行截断！");
        for (int i = 0; i < g_free.cols(); i++)
        {
            if (!std::isfinite(g_free(0, i)))
                g_free(0, i) = 0.0;
            if (!std::isfinite(g_free(1, i)))
                g_free(1, i) = 0.0;
        }
    }

    // 拷贝回 C 数组
    Eigen::Map<Eigen::MatrixXd>(grad, 2, n / 2) = g_free;
    return cost;
}
// ============================================================================
//[窥探神器] L-BFGS 内部进度监视器
// ============================================================================
int BsplineOptimizer::_progress(void *instance, const lbfgsfloatval_t *x, const lbfgsfloatval_t *g, const lbfgsfloatval_t fx,
                                const lbfgsfloatval_t xnorm, const lbfgsfloatval_t gnorm, const lbfgsfloatval_t step,
                                int n, int k, int ls)
{
    // 只打印前几次和关键次，防止刷屏
    if (k <= 3 || k % 10 == 0)
    {
        ROS_INFO("[L-BFGS 内部] Iter %d: 代价 fx=%.4f, 梯度范数 ||g||=%.4f, 步长 step=%.6f", k, fx, gnorm, step);
    }
    return 0; // 返回 0 表示继续优化
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
    
    // [调试] 打印各项代价
    static int call_count = 0;
    if (call_count < 5) {  // 只打印前 5 次调用
        ROS_INFO("[Optimizer] [Cost #%d] 总代价：%.2f = 平滑%.2f + 碰撞%.2f + 动力学%.2f", 
                 call_count, f_combine, cost_smooth, cost_collision, cost_feasibility);
        ROS_INFO("[Optimizer] [Grad #%d] 梯度范数：%.4f", call_count, g_free.norm());
        call_count++;
    }
}

// -------------------------------------------------------------------------
// 优化主函数监控增强
// -------------------------------------------------------------------------
bool BsplineOptimizer::optimize(Eigen::MatrixXd &ctrl_pts, double ts)
{
    pt_num_ = ctrl_pts.cols();
    bspline_interval_ = ts;
    control_points_ = ctrl_pts;

    int free_num = pt_num_ - 6;
    if (free_num <= 0)
    {
        ROS_INFO("[Optimizer] 路径太短，只有端点，跳过优化。");
        return true;
    }

    // ===================== 调试日志：打印所有控制点状态 =====================
    ROS_INFO("[Optimizer] ========== 控制点调试信息 ==========");
    ROS_INFO("[Optimizer] 控制点总数：%d, 自由点数：%d (索引 3 到 %d)", pt_num_, free_num, pt_num_-4);
    ROS_INFO("[Optimizer] 固定起点：控制点 0,1,2 | 固定终点：控制点 %d,%d,%d", pt_num_-3, pt_num_-2, pt_num_-1);
    for (int i = 0; i < pt_num_; ++i)
    {
        Eigen::Vector2d pt = ctrl_pts.col(i);
        bool occupied = grid_map_->isOccupied(pt);
        const char* status = occupied ? "障碍物❌" : "安全✓";
        const char* type = (i < 3 || i >= pt_num_-3) ? "固定" : "自由";
        ROS_INFO("[Optimizer] 控制点 %d [%s]: (%.3f, %.3f) -> %s", i, type, pt.x(), pt.y(), status);
    }
    ROS_INFO("[Optimizer] ======== 调试信息结束 ========");
    // ========================================================================

    // 提取自由控制点（索引 3 到 pt_num-4）
    Eigen::MatrixXd q_free = ctrl_pts.block(0, 3, 2, free_num);

    // ========================================================================
    // [史诗级修复]：放弃 std::vector，使用 lbfgs_malloc 保证 16字节内存对齐！
    // 彻底消灭 SSE 指令集不兼容导致的随机崩溃和 -1001 错误！
    // ========================================================================
    int var_num = free_num * 2;
    lbfgsfloatval_t *x_ptr = lbfgs_malloc(var_num);
    if (x_ptr == NULL)
    {
        ROS_ERROR("Failed to allocate memory for L-BFGS!");
        return false;
    }
    // 拷贝初始控制点到对齐内存中
    Eigen::Map<Eigen::VectorXd>(x_ptr, var_num) = Eigen::Map<const Eigen::VectorXd>(q_free.data(), q_free.size());

    lbfgs_parameter_t lbfgs_params;
    lbfgs_parameter_init(&lbfgs_params);
    lbfgs_params.max_iterations = param_.max_iteration_num;
    lbfgs_params.epsilon = 1e-4; // 停止阈值
    lbfgs_params.past = 3;

    double final_cost;
    ROS_INFO("[Optimizer] 启动 L-BFGS，自由点数：%d", free_num);

    // 传入我们新写的 _progress 监视器
    int ret = lbfgs(
        var_num, x_vec, &final_cost,
        BsplineOptimizer::costFunction,
        BsplineOptimizer::_progress,
        this, &lbfgs_params);

    bool success = false;
    if (ret >= 0 || ret == LBFGS_ALREADY_MINIMIZED || ret == LBFGSERR_MAXIMUMITERATION)
    {
        // 将优化结果拷贝回 Eigen 矩阵
        Eigen::MatrixXd q_free_opt = Eigen::Map<const Eigen::MatrixXd>(x_ptr, 2, free_num);
        ctrl_pts.block(0, 3, 2, free_num) = q_free_opt;
        ROS_INFO("[Optimizer] ✅ 优化成功！最终代价：%.2f (返回状态 %d)", final_cost, ret);
        success = true;
    }
    else
    {
        ROS_ERROR("[Optimizer] ❌ L-BFGS 失败！错误码：%d", ret);
        success = false;
    }
    // [极其重要]：释放对齐内存，防止内存泄漏！
    lbfgs_free(x_ptr);
    return success;
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

// -------------------------------------------------------------------------
// [核心数学修复] 碰撞代价与梯度 (真正的物理真理)
// -------------------------------------------------------------------------
void BsplineOptimizer::calcCollisionCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
{
    cost = 0.0;
    gradient.setZero();

    Eigen::Vector2d grad_dir; // 指向最近安全白格子的逃生方向
    double depth;             // 陷入障碍物的深度 (米)

    for (int i = 0; i < pt_num_; ++i)
    {
        // 如果返回 true，说明该控制点已经处于危险的黑块中！
        if (grid_map_->getObstacleGradient(q.col(i), grad_dir, depth))
        {
            // 【数学真理】：depth 就是陷入深度，陷得越深，我们要给的代价必须越大！
            // 代价 = 权重 * depth^2
            cost += param_.weight_collision * depth * depth;

            // 【梯度推导】：
            // d(Cost)/d(P_i) = 2 * weight * depth * d(depth)/d(P_i)
            // 因为顺着 grad_dir 走，能逃离黑块（让 depth 减小）。
            // 所以深度的下降梯度是 -grad_dir。
            // 所以总梯度是负的，完美闭环！
            gradient.col(i) += -2.0 * param_.weight_collision * depth * grad_dir;
        }
    }
}

void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
{
    if (bspline_interval_ < 1e-4)
        return; //[防 NaN 补丁]
    
    double max_speed_sq = param_.max_vel * param_.max_vel;
    
    for (int i = 0; i < pt_num_ - 1; i++)
    {
        // 速度 = 位移 / dt
        Eigen::Vector2d V = (q.col(i + 1) - q.col(i)) / bspline_interval_;
        double speed_sq = V.squaredNorm();

        if (speed_sq > max_speed_sq)
        {
            // 代价 = 权重 * (速度平方超标量)^2
            double penalty = speed_sq - max_speed_sq;
            cost += param_.weight_feasibility * penalty * penalty;
            
            // 梯度计算：
            // cost = w * (v^2 - vmax^2)^2
            // d(cost)/d(p_i) = 2 * w * (v^2 - vmax^2) * d(v^2)/d(p_i)
            // d(v^2)/d(p_i) = 2 * v * d(v)/d(p_i) = 2 * v * (-1/dt)
            // 所以：d(cost)/d(p_i) = -4 * w * penalty * V / dt
            Eigen::Vector2d grad_term = 4.0 * param_.weight_feasibility * penalty * V / bspline_interval_;
            gradient.col(i) += -grad_term;
            gradient.col(i + 1) += grad_term;
        }
    }
}

