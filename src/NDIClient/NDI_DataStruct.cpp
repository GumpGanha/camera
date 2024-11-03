#include<NDI_DataStruct.h>

NDIDataStruct::NDIDataStruct() :Robot_EndEffctor_(Eigen::Matrix4d::Identity())
, Head_Center_(Eigen::Matrix4d::Identity())
, Target_to_Center_(Eigen::Matrix4d::Identity())
, Aim_Joint_(0, 0, 0, 0, 0, 0)
, Target3d_to_Center(80, 80, 90)
, command_(PlanCommand::Reset) {
}

NDIDataStruct::~NDIDataStruct() {
}

void NDIDataStruct::set_Robot_EndEffctor(Position position3d) {
    // 使用 Eigen::Affine3d 创建变换矩阵
    Eigen::Affine3d transform = Eigen::Affine3d::Identity(); // 创建单位矩阵
     // 设置平移部分
    transform.translation() = position3d.position_;
    // 设置旋转部分
    transform.rotate(position3d.orientation_);
    Robot_EndEffctor_ = transform.matrix();
}

void NDIDataStruct::set_Head_Center(Position position3d) {
    // 使用 Eigen::Affine3d 创建变换矩阵
    Eigen::Affine3d transform = Eigen::Affine3d::Identity(); // 创建单位矩阵
     // 设置平移部分
    transform.translation() = position3d.position_;
    // 设置旋转部分
    transform.rotate(position3d.orientation_);
    Head_Center_ = transform.matrix();
}

std::string NDIDataStruct::planCommandToString(const PlanCommand cmd) {
    switch (cmd) {
    case PlanCommand::Reset: return "Reset";
    case PlanCommand::SimulationPlan: return "Simulation Plan";
    case PlanCommand::PlanMove: return "Plan Move";
    case PlanCommand::JointMove: return "Joint Move";
    default: return "Unknown";
    }
}

//Eigen::Matrix4d NDIDataStruct::get_Robot_EndEffctor() {
//    return Robot_EndEffctor_;
//}

Position::Position()
    :position_(0, 0, 0)
    , orientation_(1, 0, 0, 0) {
}

Position::~Position() {
}

void Position::set_position(float x, float y, float z) {
    position_.x() = x;
    position_.y() = y;
    position_.z() = z;
}

void Position::set_orientation(float q0, float qx, float qy, float qz) {
    orientation_.w() = q0;
    orientation_.x() = qx;
    orientation_.y() = qy;
    orientation_.z() = qz;
}