#pragma once

#include<iostream>
#include <string>
#include <Eigen/Dense>
#include<vector>
using std::string;
using std::vector;
// 定义枚举类表示命令
enum class PlanCommand {
    Reset = 0,
    SimulationPlan,
    PlanMove,
    JointMove
};

//// 函数：将枚举转换为字符串
//std::string planCommandToString(PlanCommand cmd) {
//    switch (cmd) {
//    case PlanCommand::Reset: return "Reset ";
//    case PlanCommand::SimulationPlan: return "Simulation Plan";
//    case PlanCommand::PlanMove: return "Plan Move";
//    case PlanCommand::JointMove: return "Joint Move";
//    default: return "Unknown";
//    }
//}

class Position {
public:
    Position();
    ~Position();
    void set_position(float x, float y, float z);
    void set_orientation(float q0, float qx, float qy, float qz);
    Eigen::Vector3d position_;
    Eigen::Quaterniond orientation_;
};

class NDIDataStruct {
public:
    NDIDataStruct();
    ~NDIDataStruct();
    void set_Robot_EndEffctor(Position  position3d);
    void set_Head_Center(Position  position3d);
    // 函数：将枚举转换为字符串
    std::string planCommandToString(const PlanCommand cmd);
    //Eigen::Matrix4d get_Robot_EndEffctor();

    Eigen::Matrix4d     Robot_EndEffctor_;
    Eigen::Matrix4d     Head_Center_;
    Eigen::Matrix4d     Target_to_Center_;
    Eigen::Matrix<double, 6, 1>      Aim_Joint_;
    Eigen::Matrix<double, 3, 1>        Target3d_to_Center;
    PlanCommand             command_;
};