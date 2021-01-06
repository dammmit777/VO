/*Feature 类
*主要信息：自身的 2D 位置
*is_outlier 为异常点的标志位
*is_on_left_image 为它是否在左侧相机提取的标志位
*通过一个 Feature 对象访问持有它的 Frame 及它对应的路标
*注意： Frame 和 MapPoint 的实际持有权归 map.h 所有，
        为避免 shared_ptr 产生循环引用，这里使用了 weak_ptr
*/
#pragma once
#ifndef MYSLAM_FEATURE_H
#define MYSLAM_FEATURE_H

#include <memory>
#include <opencv2/features2d.hpp>
#include "myslam/common_include.h"

namespace myslam
{
struct Frame;
struct MapPoint;

//--2D 特征点
//在三角化之后会被关联一个地图点
struct Feature
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame_;        //持有该特征的帧
    cv::KeyPoint position_;             //2D 提取位置
    std::weak_ptr<MapPoint> map_point_; //关联地图点

    bool is_outlier_ = false;      //是否为异常点
    bool is_on_left_image_ = true; //标识是否提在左图， false 为右图

public:
    Feature() {}

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
        : frame_(frame), position_(kp) {}
};
} // namespace myslam

#endif // !MYSLAM_FEATURE_H
