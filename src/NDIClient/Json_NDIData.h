#pragma once
#include<iostream>
#include <Eigen/Dense>
#include<string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include"boost/filesystem.hpp"
#include <boost/iostreams/device/mapped_file.hpp>

#include <NDI_DataStruct.h>

using std::string;
const std::string Head_NDI = "XsReceiveHead";
const std::string Tail_NDI = "XsReceiveTail";

class Json_NDIData {
public:
    Json_NDIData();
    ~Json_NDIData();
    string create_json(NDIDataStruct& data);
    string add_check(std::string& str);
    // ½« Eigen ¾ØÕó×ª»»Îª ptree
    boost::property_tree::ptree eigenToPtree(const Eigen::MatrixXd& matrix);
private:
};
