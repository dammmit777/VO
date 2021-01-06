#pragma once
#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H

#include "myslam/common_include.h"

namespace myslam
{
/**
 * 配置类，使用 SetParameterFile 确定配置文件
 * 然后用 Get 得到对应值
 * 单例模式
 */
class Config
{
private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config() {} //使一个单例有私有构造函数

public:
    ~Config(); //解构时关闭文件

    //设置一个新的配置文件
    static bool SetParameterFile(const std::string &filename);

    //访问参数值
    template <typename T>
    static T Get(const std::string &key)
    {
        return T(Config::config_->file_[key]);
    }
};
} // namespace myslam

#endif // !MYSLAM_CONFIG_H
