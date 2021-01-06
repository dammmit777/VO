#ifndef MYSLAM_DATASET_H
#define MYSLAM_DATASET_H

#include "myslam/common_include.h"
#include "myslam/camera.h"
#include "myslam/frame.h"

namespace myslam
{
/**
 * 数据集读取
 * 构造时传入配置文件路径，配置文件的 dataset_dir 为数据集路径
 * Init 之后可获得相机和下一帧图像
*/
class Dataset
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    typedef std::shared_ptr<Dataset> Ptr;

    Dataset(const std::string &dataset_path);

    //初始化，返回是否成功
    bool Init();

    //创建并返回包含立体图像的下一帧
    Frame::Ptr NextFrame();

    //通过 id 获取相机
    Camera::Ptr GetCamera(int camera_id) const { return cameras_.at(camera_id); }

private:
    std::string dataset_path_;
    int current_image_index_ = 0;
    std::vector<Camera::Ptr> cameras_;
};

} // namespace myslam

#endif // !MYSLAM_DATASET_H
