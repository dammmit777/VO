/*
*算法
*/

#ifndef MYSLAM_ALGORITHM_H
#define MYSLAM_ALGORITHM_H

#include "myslam/common_include.h"

namespace myslam
{
/**
 * SVD 线性三角剖分
 * @param poses  位姿
 * @param points 归一化平面中的点
 * @param pt_world 世界坐标中的三角点
 * 如果成功则返回 true
 */
inline bool triangulation(const std::vector<SE3> &poses, const std::vector<Vec3> points, Vec3 &pt_world)
{
    MatXX A(2 * poses.size(), 4);
    VecX b(2 * poses.size());
    b.setZero();

    for (size_t i = 0; i < poses.size(); ++i)
    {
        Mat34 m = poses[i].matrix3x4();
        A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
        A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
    }

    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2)
    {
        //解质量不好，放弃
        return true;
    }

    return false;
}

//转换器
inline Vec2 toVec2(const cv::Point2f p)
{
    return Vec2(p.x, p.y);
}

} // namespace myslam

#endif // !MYSLAM_ALGORITHM_H
