#pragma once
#ifndef MYSLAM_VISUAL_ODOMETRY_H
#define MYSLAM_VISUAL_ODOMETRY_H

#include "myslam/common_include.h"
#include "myslam/frontend.h"
#include "myslam/backend.h"
#include "myslam/dataset.h"
#include "myslam/viewer.h"

namespace myslam
{
/**
 * VO 对外接口
*/
class VisualOdometry
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef std::shared_ptr<VisualOdometry> Ptr;

    //带有配置文件的构造函数
    VisualOdometry(std::string &config_path);

    /**
     * 在运行之前进行初始化操作
     * @return true if success
    */
    bool Init();

    /**
    * 在数据集中启动视觉里程计
    */
    void Run();

    //在数据集中迈出一步
    bool Step();

    //获取前端状态
    FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

private:
    bool inited_ = false;
    std::string config_file_path_;

    Frontend::Ptr frontend_ = nullptr;
    Backend::Ptr backend_ = nullptr;
    Map::Ptr map_ = nullptr;
    Viewer::Ptr viewer_ = nullptr;

    //dataset
    Dataset::Ptr dataset_ = nullptr;
};

} // namespace myslam

#endif // !MYSLAM_VISUAL_ODOMETRY_H
