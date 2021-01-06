#pragma once
#ifndef MYSLAM_CAMERA_H
#define MYSLAM_CAMERA_A

#include "myslam/common_include.h"

namespace myslam
{
//针孔立体摄像机型号
class Camera
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef std::shared_ptr<Camera> Ptr;

    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0,
           baseline_ = 0; //相机内参
    SE3 pose_;            //外在位姿，从立体相机到单相机
    SE3 pose_inv_;        //外在位姿的逆

    Camera();

    Camera(double fx, double fy, double cx, double cy, double baseline, const SE3 &pose)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose)
    {
        pose_inv_ = pose_.inverse();
    }

    SE3 pose() const { return pose_; }

    //返回内在矩阵
    Mat33 K() const
    {
        Mat33 k;
        k << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1;
        return k;
    }

    //坐标变换：世界，相机，像素
    Vec3 world2camera(const Vec3 &p_w, const SE3 &T_c_w);
    Vec3 camera2world(const Vec3 &p_c, const SE3 &T_c_w);
    Vec2 camera2pixel(const Vec3 &p_c);
    Vec3 pixel2camera(const Vec2 &p_p, double depth = 1);
    Vec3 pixel2world(const Vec2 &p_p, const SE3 &T_c_w, double depth = 1);
    Vec2 world2pixel(const Vec3 &p_w, const SE3 &T_c_w);
};
} // namespace myslam

#endif // !MYSLAM_CAMERA_H
