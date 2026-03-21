#include "mission_controller.h"
#include <iostream>

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "mission_node");
    ros::NodeHandle nh("~");

    // 实例化 Boss
    MissionController boss;
    boss.init(nh);

    ros::Rate rate(20.0);

    // 1. 等待飞控连接 (防死锁心跳)
    ROS_INFO("等待飞控连接...");
    while (ros::ok() && !boss.isConnected())
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("飞控已连接！");

    // 2. 比赛发令枪：按 1 启动
    int input = 0;
    std::cout << "===========================" << std::endl;
    std::cout << "请输入 1 确认起飞并开始比赛任务: ";
    std::cin >> input;
    if (input != 1)
    {
        ROS_WARN("任务取消。");
        return 0;
    }

    // 3. 启动主循环
    ROS_INFO(">>> 比赛正式开始！ <<<");
    int flag=0;
    while (ros::ok())
    {if(flag==0)
        {
            ROS_INFO(">>> happy begin 3！<<<");
            flag=1;
        }
        ros::spinOnce();
        boss.tick(); // 驱动任务状态机
        rate.sleep();
    }

    return 0;
}