/*帧
*路标
*特征
*这些数据被多个线程访问或修改，需要在关键部分加入线程锁。
*定义 Frame 含有 id、位姿、图像及左右图像中的特征点，
*其中 Pose 会被前后端同时设置或访问，所以定义 Pose 的 Set 和 Get 函数，在函数内加锁
*Frame 可由静态函数构建，在静态函数中可以自动分配 id
*/
#pragma once
#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "myslam/camera.h"
#include "myslam/common_include.h"

namespace myslam
{
//声明
struct MapPoint;
struct Feature;

//--帧
//每一帧分配独立 id， 关键帧分配关键帧 ID
struct Frame
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef std::shared_ptr<Frame> Ptr;

    unsigned long id_ = 0;          //当前帧的 id
    unsigned long keyframe_id_ = 0; //关键帧的 id
    bool is_keyframe_ = false;      //是否为关键帧
    double time_stamp_;             //时间戳，暂不使用
    SE3 pose_;                      // Tcw 形式的 Pose
    std::mutex pose_mutex_;         // Pose 数据锁
    cv::Mat left_img_, right_img_;  // stereo images

    //左侧图像中提取的特征
    std::vector<std::shared_ptr<Feature>> features_left_;

    //右侧图片中的对应特征，如果没有对应的，则设置为 nullptr
    std::vector<std::shared_ptr<Feature>> features_right_;

public: //数据成员
    Frame() {}

    Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right);

    //设置并且获取位姿，保护线程
    SE3 Pose()
    {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }

    void SetPose(const SE3 &pose)
    {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }

    //设置关键帧并分配关键帧 id
    void SetKeyFrame();

    //工厂构建模式，分配 id
    static std::shared_ptr<Frame> CreateFrame();
};

} // namespace myslam
#endif // !MYSLAM_FRAME_H
