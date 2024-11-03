#include"Json_NDIData.h"

Json_NDIData::Json_NDIData() {
}

Json_NDIData::~Json_NDIData() {
}

string Json_NDIData::create_json(NDIDataStruct& data) {
    boost::property_tree::ptree pt, pt0;
    //�������
    pt0.put_child("Position_Matrix1", eigenToPtree(data.Robot_EndEffctor_));
    pt0.put_child("Position_Matrix2", eigenToPtree(data.Head_Center_));
    pt0.put_child("Position_Matrix3", eigenToPtree(data.Target_to_Center_));
    pt0.put_child("Target_Center", eigenToPtree(data.Target3d_to_Center));
    pt0.put_child("Joint_Matrix", eigenToPtree(data.Aim_Joint_));
    //��������ö������
    PlanCommand  temp = data.command_;
    string command = data.planCommandToString(temp);
    pt0.put("Command", string(command));
    //����Receive_msg ���а�װ
    pt.put_child("Receive_msg", pt0);
    //��jsonת��Ϊ�ַ���
    std::stringstream ss;
    boost::property_tree::write_json(ss, pt);
    // �� ptree д�� JSON �ļ�
    //string filename = "E:/laborary/TMS project/NDISendVersion/nditext.json";
    //boost::property_tree::write_json(filename, pt);
    return ss.str();
}

string Json_NDIData::add_check(std::string& str) {
    //������ ����ͷ��β����ΪУ��
    //str.append(Tail_NDI);
    //str.insert(0, Head_NDI);
    //return str;
    std::string result = Head_NDI;    // Ԥ�ȷ����ͷ���ռ�
    result += str;                    // ׷��ԭʼ�ַ���
    result += Tail_NDI;               // ׷��β��
    return result;
}

boost::property_tree::ptree Json_NDIData::eigenToPtree(const Eigen::MatrixXd& matrix) {
    boost::property_tree::ptree pt;

    // ���������ÿ��Ԫ�أ��洢�� ptree ��
    for (int j = 0; j < matrix.cols(); ++j) {
        boost::property_tree::ptree row;
        for (int i = 0; i < matrix.rows(); ++i) {
            row.put("", matrix(i, j));
            pt.push_back(std::make_pair("", row));
        }
    }

    return pt;
}