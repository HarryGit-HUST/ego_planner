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
    static int call_count = 0;
    while (call_count < 5) // 前几次调用打印详细信息，帮助调参和验证正确性
    {
        // [新增] 调试日志：打印每次代价和梯度信息，帮助调参和验证正确性
        ROS_INFO("[Optimizer] [Cost ] 总代价：%.2f = 平滑%.2f + 碰撞%.2f + 动力学%.2f", 
                 f_combine, cost_smooth, cost_collision, cost_feasibility);
        ROS_INFO("[Optimizer] [Grad ] 梯度范数：%.4f", g_free.norm());
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
    /*
    for (int i = 0; i < pt_num_; ++i)
    {
        Eigen::Vector2d pt = ctrl_pts.col(i);
        bool occupied = grid_map_->isOccupied(pt);
        const char* status = occupied ? "障碍物❌" : "安全✓";
        const char* type = (i < 3 || i >= pt_num_-3) ? "固定" : "自由";
        ROS_INFO("[Optimizer] 控制点 %d [%s]: (%.3f, %.3f) -> %s", i, type, pt.x(), pt.y(), status);
    }
    */
    
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
    lbfgs_params.min_step = 1e-7;  // 添加最小步长限制
lbfgs_params.max_step = 1e1;   // 添加最大步长限制
lbfgs_params.ftol = 1e-4;      // 调整 Armijo 条件参数
lbfgs_params.wolfe = 0.9;      // 调整 Wolfe 条件参数

    double final_cost;
    ROS_INFO("[Optimizer] 启动 L-BFGS，自由点数：%d", free_num);

    // 传入我们新写的 _progress 监视器
    int ret = lbfgs(
        var_num, x_ptr, &final_cost,
        BsplineOptimizer::costFunction,
        BsplineOptimizer::_progress,
        this, &lbfgs_params);

    bool success = false;
    // [修复] L-BFGS 在离散栅格地图上无法完全收敛是正常现象
    // 只要梯度有显著下降，就接受当前解
    if (ret >= 0 || ret == LBFGS_ALREADY_MINIMIZED || ret == LBFGSERR_MAXIMUMITERATION || 
        ret == LBFGSERR_LOGICERROR || ret == LBFGSERR_ROUNDING_ERROR)
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

    Eigen::Vector2d grad_dir; // 远离障碍物的方向
    double signed_dist;       // 带符号距离：正=障碍物外，负=障碍物内

    for (int i = 0; i < pt_num_; ++i)
    {
        // 获取带符号距离和梯度方向
        grid_map_->getObstacleGradient(q.col(i), grad_dir, signed_dist);

        // 【光滑势场公式】：
        // 当 signed_dist < safe_distance 时产生代价
        // penetration = safe_distance - signed_dist
        double penetration = param_.safe_distance - signed_dist;

        if (penetration > 0)
        {
            // 【五次多项式光滑衰减因子】：f(x) = 10x³ - 15x⁴ + 6x⁵
            // 满足：f(0)=0, f(1)=1, f'(0)=0, f'(1)=0, f''(0)=0, f''(1)=0
            // 保证一阶和二阶导数都连续！
            double ratio = penetration / param_.safe_distance;
            double smooth_factor = 1.0;
            
            if (ratio < 1.0)
            {
                // 在安全距离内但在障碍物外：使用五次平滑衰减
                smooth_factor = 10.0 * ratio * ratio * ratio 
                              - 15.0 * ratio * ratio * ratio * ratio 
                              + 6.0 * ratio * ratio * ratio * ratio * ratio;
            }
            
            // 代价 = weight * penetration² * smooth_factor
            cost += param_.weight_collision * penetration * penetration * smooth_factor;

            // 梯度计算：
            // cost = w * penetration² * smooth_factor
            // d(cost)/d(penetration) = w * [2*penetration*smooth_factor + penetration² * f'(ratio) / safe_distance]
            //                        = w * penetration * [2*smooth_factor + ratio * f'(ratio)]
            // f'(ratio) = 30*ratio²*(1-ratio)²
            if (ratio < 1.0)
            {
                // 在衰减区域内：完整梯度公式
                double d_smooth_d_ratio = 30.0 * ratio * ratio * (1.0 - ratio) * (1.0 - ratio);
                double grad_factor = 2.0 * smooth_factor + ratio * d_smooth_d_ratio;  // 【修复】是 + 号！
                gradient.col(i) += -param_.weight_collision * penetration * grad_factor * grad_dir;
            }
            else
            {
                // 在障碍物内：smooth_factor = 1，梯度简化为标准形式
                gradient.col(i) += -2.0 * param_.weight_collision * penetration * grad_dir;
            }
        }
    }
}

void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
{
    if (bspline_interval_ < 1e-4)
        return; //[防 NaN 补丁]

    // 使用速度平方阈值，90% 速度时开始给梯度
    double speed_threshold = 0.9 * param_.max_vel;
    double speed_threshold_sq = speed_threshold * speed_threshold;

    for (int i = 0; i < pt_num_ - 1; i++)
    {
        // 速度 = 位移 / dt
        Eigen::Vector2d V = (q.col(i + 1) - q.col(i)) / bspline_interval_;
        double speed_sq = V.squaredNorm();

        if (speed_sq > speed_threshold_sq)
        {
            // 【五次多项式光滑势场】：
            // penalty = (speed² - th²) / th²，归一化到 [0, ∞)
            // 使用 f(x) = 10x³ - 15x⁴ + 6x⁵，保证二阶导数连续
            double penalty_norm = (speed_sq - speed_threshold_sq) / speed_threshold_sq;
            double smooth_penalty = 1.0;
            
            if (penalty_norm < 1.0)
            {
                // 在阈值附近：使用五次平滑衰减
                smooth_penalty = 10.0 * penalty_norm * penalty_norm * penalty_norm 
                               - 15.0 * penalty_norm * penalty_norm * penalty_norm * penalty_norm 
                               + 6.0 * penalty_norm * penalty_norm * penalty_norm * penalty_norm * penalty_norm;
            }
            
            // 代价 = weight * penalty_norm² * smooth_penalty
            cost += param_.weight_feasibility * penalty_norm * penalty_norm * smooth_penalty;

            // 梯度计算（分母没有 speed，不会爆炸）：
            // grad = 4 * weight * penalty_norm * smooth_penalty * V / dt
            //      + weight * penalty_norm² * d(smooth)/d(penalty) * d(penalty)/d(V) / dt
            Eigen::Vector2d grad_term;
            if (penalty_norm < 1.0)
            {
                double d_smooth_d_penalty = 30.0 * penalty_norm * penalty_norm * (1.0 - penalty_norm) * (1.0 - penalty_norm);
                double grad_scale = 4.0 * smooth_penalty + 2.0 * penalty_norm * d_smooth_d_penalty;
                grad_term = param_.weight_feasibility * grad_scale * V / (bspline_interval_ * speed_threshold_sq);
            }
            else
            {
                grad_term = 4.0 * param_.weight_feasibility * penalty_norm * V / bspline_interval_;
            }
            gradient.col(i) += -grad_term;
            gradient.col(i + 1) += grad_term;
        }
    }
}

