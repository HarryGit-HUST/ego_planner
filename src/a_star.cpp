#include "a_star.h"
#include "grid_map.h" // 必须包含测绘部的头文件
#include <algorithm>
#include <cmath>

AStar::AStar() {}
AStar::~AStar() {}

void AStar::init(std::shared_ptr<GridMap> grid_map)
{
    grid_map_ = grid_map;
}

// ============================================================================
// [核心引擎] A* 主搜索算法
// ============================================================================
bool AStar::search(const Eigen::Vector2d &start_pt, const Eigen::Vector2d &target_pt, std::vector<Eigen::Vector2d> &path)
{
    // 1. 安全拷贝与起点/终点校验 (防止卡在墙内)
    Eigen::Vector2d safe_start = start_pt;
    Eigen::Vector2d safe_target = target_pt;
    findNearestFreePoint(safe_start);
    findNearestFreePoint(safe_target);

    // 2. 物理坐标转为栅格索引
    Eigen::Vector2i start_idx, target_idx;
    if (!grid_map_->posToIndex(safe_start, start_idx.x(), start_idx.y()) ||
        !grid_map_->posToIndex(safe_target, target_idx.x(), target_idx.y()))
    {
        return false; // 起终点完全越界
    }

    // 获取地图尺寸
    int grid_w = grid_map_->getGridW();
    int grid_h = grid_map_->getGridH();

    // =========================================================
    // 3. 建立 OpenSet 和 O(1) 极速哈希池 (NodePool)
    // =========================================================
    std::priority_queue<AStarNode *, std::vector<AStarNode *>, NodeComparator> open_set;

    // 【内存安全方案】：用指针池。大小固定，绝对不 resize。
    std::vector<AStarNode *> node_pool(grid_w * grid_h, nullptr);
    // 专门用来记录 new 出来的节点，方便函数结束时一键销毁，防内存泄漏！
    std::vector<AStarNode *> allocated_nodes;

    // =========================================================
    // 4. 初始化起点
    // =========================================================
    AStarNode *start_node = new AStarNode(start_idx, safe_start);
    start_node->g_score = 0;
    start_node->f_score = getHeuristic(safe_start, safe_target, safe_start);
    start_node->state = 1; // 1 = 放入了 Open 表

    open_set.push(start_node);
    node_pool[start_idx.x() + start_idx.y() * grid_w] = start_node;
    allocated_nodes.push_back(start_node); // 登记，准备最后销毁

    // =========================================================
    // 5. 开启寻路主循环 (The Expansion Loop)
    // =========================================================
    AStarNode *final_node = nullptr;

    while (!open_set.empty())
    {
        AStarNode *cur_node = open_set.top();
        open_set.pop();

        // 【极其重要】：Lazy Deletion 懒惰删除
        // 如果这个点之前已经被扩展过了（后来又有更优的路径进队），直接跳过！
        if (cur_node->state == 2)
            continue;

        // 到达终点，寻路成功！
        if (cur_node->index == target_idx)
        {
            final_node = cur_node;
            break;
        }

        // 标记为已访问 (Closed 表)
        cur_node->state = 2;

        // 8 邻域搜索
        int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
        int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

        for (int i = 0; i < 8; i++)
        {
            int nx = cur_node->index.x() + dx[i];
            int ny = cur_node->index.y() + dy[i];

            // 越界或撞墙，跳过
            if (nx < 0 || nx >= grid_w || ny < 0 || ny >= grid_h)
                continue;
            if (grid_map_->isOccupied(nx, ny))
                continue;

            int neighbor_idx = nx + ny * grid_w; // 1D 极速索引
            AStarNode *neighbor_node = node_pool[neighbor_idx];

            // 如果该邻居已经在 Closed 表，不走回头路
            if (neighbor_node != nullptr && neighbor_node->state == 2)
                continue;

            // 计算新的 g 和 f代价
            double edge_cost = (i < 4) ? 1.0 : 1.41421356; // 上下左右 1，斜角 根号2
            double tentative_g_score = cur_node->g_score + edge_cost;

            // 获取邻居的物理坐标
            Eigen::Vector2d neighbor_pos;
            grid_map_->indexToPos(nx, ny, neighbor_pos);

            // 如果节点还不存在，实例化它
            if (neighbor_node == nullptr)
            {
                neighbor_node = new AStarNode(Eigen::Vector2i(nx, ny), neighbor_pos);
                node_pool[neighbor_idx] = neighbor_node;
                allocated_nodes.push_back(neighbor_node); // 登记销毁
            }

            // 发现更优路径，更新之
            if (tentative_g_score < neighbor_node->g_score)
            {
                neighbor_node->parent = cur_node;
                neighbor_node->g_score = tentative_g_score;
                neighbor_node->f_score = tentative_g_score + getHeuristic(neighbor_pos, safe_target, safe_start);

                // 只要更新了，就压入优先队列 (即使它之前在队列里，也会因为 f_score 更小而在前面先弹出)
                neighbor_node->state = 1;
                open_set.push(neighbor_node);
            }
        }
    }

    // =========================================================
    // 6. 回溯生成原始路径 & 内存清理
    // =========================================================
    bool success = false;
    std::vector<Eigen::Vector2d> raw_path;

    if (final_node != nullptr)
    {
        AStarNode *cur = final_node;
        while (cur != nullptr)
        {
            raw_path.push_back(cur->pos);
            cur = cur->parent;
        }
        std::reverse(raw_path.begin(), raw_path.end());

        // 7. 视距法极简剪裁 (去除锯齿，为 B 样条打基础)
        simplifyPath(raw_path, path);
        success = true;
    }

    // 【内存扫除】：遍历记录本，把 new 出来的几百个节点一键 delete，滴水不漏！
    for (auto node : allocated_nodes)
    {
        delete node;
    }

    return success;
}

// ============================================================================
//[数学工具] 启发式代价与视距检测
// ============================================================================

double AStar::getHeuristic(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &start_pt)
{
    // 1. 基础启发式：欧几里得距离
    double h = (p1 - p2).norm();

    // 2. 叉乘打破对称性 (Cross-Product Tie-breaker) —— 工业级提速秘籍
    // 计算当前点到“起点-终点连线”的偏离程度，逼迫 A* 走最直的线
    double dx1 = p1.x() - p2.x();
    double dy1 = p1.y() - p2.y();
    double dx2 = start_pt.x() - p2.x();
    double dy2 = start_pt.y() - p2.y();
    double cross = std::abs(dx1 * dy2 - dx2 * dy1);

    return h + cross * 0.001; // 加上一个极小的权重惩罚偏离航线的点
}

void AStar::simplifyPath(const std::vector<Eigen::Vector2d> &raw_path, std::vector<Eigen::Vector2d> &simplified_path)
{
    if (raw_path.empty())
        return;

    simplified_path.clear();
    simplified_path.push_back(raw_path[0]);

    int last_idx = 0;
    for (int i = 1; i < raw_path.size(); i++)
    {
        // 如果最后确定的点，到当前点被挡住了，说明必须要在前一个点拐弯！
        if (!checkLineOfSight(simplified_path.back(), raw_path[i]))
        {
            simplified_path.push_back(raw_path[i - 1]);
            last_idx = i - 1;
        }
    }
    // 强制压入终点
    if (simplified_path.back() != raw_path.back())
    {
        simplified_path.push_back(raw_path.back());
    }
}

bool AStar::checkLineOfSight(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2)
{
    Eigen::Vector2d dir = p2 - p1;
    double dist = dir.norm();
    if (dist < 1e-3)
        return true;

    dir.normalize();
    //[核心修复] 步长设为栅格的一半 (0.05m)，确保激光射线密集扫过，不漏过任何黑边！
    double step = 0.05;

    for (double d = 0; d <= dist; d += step)
    {
        Eigen::Vector2d pt = p1 + dir * d;
        if (grid_map_->isOccupied(pt))
            return false;
    }
    return true;
}
bool AStar::findNearestFreePoint(Eigen::Vector2d &pt)
{
    return grid_map_->searchNearestFreeSpace(pt, pt);
}