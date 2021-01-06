/*地图点类
*主要信息：自身的 3D 位置，即 pos_ 变量，需要对他上锁
*它的 observation_ 变量记录了自己被哪些 Feature 观察
*Feature 可能被判定为 outlier，所以 observation 部分发生改动时也需要锁定
*/
#pragma once
#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam
{
struct Frame;
struct Feature;

//--路标点类
//特征点在三角化之后形成路标点
struct MapPoint
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef std::shared_ptr<MapPoint> Ptr;

    unsigned long id_ = 0;     //ID
    bool is_outlier_ = false; //是否为异常点
    Vec3 pos_ = Vec3::Zero(); //世界坐标系中的坐标
    std::mutex data_mutex_;
    int observed_times_ = 0; //被特征匹配算法观测到
    std::list<std::weak_ptr<Feature>> observations_;

    MapPoint() {}
    MapPoint(long id, Vec3 position);

    Vec3 Pos()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return pos_;
    }

    void SetPos(const Vec3 &pos)
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        pos_ = pos;
    }

    void AddObservation(std::shared_ptr<Feature> feature)
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        observations_.push_back(feature);
        observed_times_++;
    }

    void RemoveObservation(std::shared_ptr<Feature> feat);

    std::list<std::weak_ptr<Feature>> GetObs()
    {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return observations_;
    }

    //工厂功能
    static MapPoint::Ptr CreateNewMappoint();
};
} // namespace myslam

#endif // !MYSLAM_MAPPOINT_H
