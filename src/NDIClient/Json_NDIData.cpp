#include"Json_NDIData.h"

Json_NDIData::Json_NDIData() {
}

Json_NDIData::~Json_NDIData() {
}

string Json_NDIData::create_json(NDIDataStruct& data) {
    boost::property_tree::ptree pt, pt0;
    //处理矩阵
    pt0.put_child("Position_Matrix1", eigenToPtree(data.Robot_EndEffctor_));
    pt0.put_child("Position_Matrix2", eigenToPtree(data.Head_Center_));
    pt0.put_child("Position_Matrix3", eigenToPtree(data.Target_to_Center_));
    pt0.put_child("Target_Center", eigenToPtree(data.Target3d_to_Center));
    pt0.put_child("Joint_Matrix", eigenToPtree(data.Aim_Joint_));
    //处理命令枚举类型
    PlanCommand  temp = data.command_;
    string command = data.planCommandToString(temp);
    pt0.put("Command", string(command));
    //加上Receive_msg 进行包装
    pt.put_child("Receive_msg", pt0);
    //将json转换为字符串
    std::stringstream ss;
    boost::property_tree::write_json(ss, pt);
    // 将 ptree 写入 JSON 文件
    //string filename = "E:/laborary/TMS project/NDISendVersion/nditext.json";
    //boost::property_tree::write_json(filename, pt);
    return ss.str();
}

string Json_NDIData::add_check(std::string& str) {
    //对数据 加入头和尾，作为校验
    //str.append(Tail_NDI);
    //str.insert(0, Head_NDI);
    //return str;
    std::string result = Head_NDI;    // 预先分配好头部空间
    result += str;                    // 追加原始字符串
    result += Tail_NDI;               // 追加尾部
    return result;
}

boost::property_tree::ptree Json_NDIData::eigenToPtree(const Eigen::MatrixXd& matrix) {
    boost::property_tree::ptree pt;

    // 遍历矩阵的每个元素，存储到 ptree 中
    for (int j = 0; j < matrix.cols(); ++j) {
        boost::property_tree::ptree row;
        for (int i = 0; i < matrix.rows(); ++i) {
            row.put("", matrix(i, j));
            pt.push_back(std::make_pair("", row));
        }
    }

    return pt;
}