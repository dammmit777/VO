#pragma once
#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include <opencv2/features2d.hpp>

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam
{
class Backend; //后端类
class Viewer;  //可视化类

enum class FrontendStatus
{
    INITING,
    TRACKING_GOOD,
    TRACKING_BAD,
    LOST
}; //前端状态：初始化，追踪成功，追踪失败，追踪丢失

/**
 * 前端
 * 估计当前帧的 Pose ,在满足关键帧条件时向地图加入关键帧并触发优化
*/
class Frontend
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef std::shared_ptr<Frontend> Ptr;

    Frontend();

    //外部接口，添加一个帧并计算其定位结果
    bool AddFrame(Frame::Ptr frame);

    //Set 函数
    void SetMap(Map::Ptr map) { map_ = map; }
    void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }
    void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }
    FrontendStatus GetStatus() const { return status_; }
    void SetCameras(Camera::Ptr left, Camera::Ptr right)
    {
        camera_left_ = left;
        camera_right_ = right;
    }

private:
    /**
     * 在正常模式下追踪
     * @return true if success
    */
    bool Track();

    /**
    * 丢失时重置
    * @return true if success
   */
    bool Reset();

    /**
     * 追踪最后一帧
     * 返回追踪点数
    */
    int TrackLastFrame();

    /**
    * 估计当前帧的位姿
    * 返回数量 num of inliers
   */
    int EstimateCurrentPose();

    /**
     * 将当前帧设置为关键帧并将其插入后端
     * @return true if success
    */
    bool InsertKeyframe();

    /**
     * 尝试使用保存在 current_frame_ 中的立体图像初始化前端
     * @return true if success
    */
    bool StereoInit();

    /**
     * 在 current_frame_ 中检测左侧图像中的特征
     * 关键点将保存在 current_frame_ 中
     * @return
    */
    int DetectFeatures();

    /**
     * 在 current_frame_ 的右图中找到相应的特征
     * 返回找到的特征数量
    */
    int FindFeaturesInRight();

    /**
     * 用单个图像构建初始地图
     * @return true if success
    */
    bool BuildInitMap();

    /**
     * 对当前帧中的 2D 点进行三角剖分
     * 返回三角点数量
    */
    int TriangulateNewPoints();

    /**
     * 将关键帧中的特征设置为对地图点的新观察
    */
    void SetObservationsForKeyFrame();

    //data
    FrontendStatus status_ = FrontendStatus::INITING;
    Frame::Ptr current_frame_ = nullptr; //当前帧
    Frame::Ptr last_frame_ = nullptr;    //上一帧
    Camera::Ptr camera_left_ = nullptr;  //左侧相机
    Camera::Ptr camera_right_ = nullptr; //右侧相机
    Map::Ptr map_ = nullptr;
    std::shared_ptr<Backend> backend_ = nullptr;
    std::shared_ptr<Viewer> viewer_ = nullptr;

    SE3 relative_motion_;      //当前帧与上一帧的相对运动，用于估计当前帧的 pose 初值
    int tracking_inliers_ = 0; //内线，用于测试关键帧

    //参数
    int num_features_ = 200;
    int num_features_init_ = 100;
    int num_features_tracking_ = 50;
    int num_features_tracking_bad_ = 20;
    int num_features_needed_for_keyframe_ = 80;

    //公用
    cv::Ptr<cv::GFTTDetector> gftt_;//OpenCV 中的特征检测器
};
} // namespace myslam

#endif // !MYSLAM_FRONTEND_H
