#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Core>          // 使用Eigen库进行矩阵计算
#include <Eigen/Geometry>      // 使用Eigen库进行几何变换
#include <unordered_map>       // 使用无序哈希表实现图像序列的存储
#include <algorithm>           // 使用sort函数排序
#include <iomanip>             // 使用iomanip库输出浮点数精度
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

struct PoseWithStamp {
    Eigen::Quaternionf rotation;   // 旋转四元数
    Eigen::Vector3f translation;   // 平移向量
    double stamp;                  // 时间戳
    int idx;                       // 索引
};

struct ImageWithPose {
    PoseWithStamp pose;            // 相机位姿
    bool valid = false;            // 是否有效
};








vector<ImageWithPose> pose_interpolation(std::vector<ImageWithPose>& image_hashtable_, std::vector<PoseWithStamp>& trajectory_)
{
    vector<ImageWithPose> poseAndtime;

    // 遍历所有的image_hashtable_元素
    for (ImageWithPose &pimg : image_hashtable_)
    {
        
        // 找到对应时间戳的trajectory_元素
        auto it = std::lower_bound(trajectory_.begin(), trajectory_.end(), pimg.pose.stamp, [](const PoseWithStamp& p, double t) { return p.stamp < t; });
       if (it == trajectory_.begin() || it == trajectory_.end())
        {
            // 如果时间戳不在trajectory_的时间戳范围内，则忽略该图像
            continue;
        }

        // 对相机位姿进行插值
        Eigen::Quaternionf q0 = (it - 1)->rotation.normalized();  // 前一帧的旋转四元数
        Eigen::Quaternionf q1 = it->rotation.normalized();        // 后一帧的旋转四元数
        float ratio = (pimg.pose.stamp - (it - 1)->stamp) / (it->stamp - (it - 1)->stamp);  // 求时间戳比例
        pimg.pose.rotation = q0.slerp(ratio, q1);                  // 旋转插值
        pimg.pose.translation = (it - 1)->translation + (it->translation - (it - 1)->translation) * ratio;  // 平移插值
        pimg.pose.stamp = (it - 1)->stamp;                         // 插值后的时间戳为前一帧的时间戳
        pimg.pose.idx = (it - 1)->idx;                             // 插值后的索引为前一帧的索引
        pimg.valid = true;

        // 将插值后的图像加入到poseAndtime向量中
        poseAndtime.push_back(pimg);
        
       
    }

   return poseAndtime;
}

int main()
{   // 读取轨迹文件
    vector<ImageWithPose> poseAndtime;
    std::vector<PoseWithStamp> trajectory_;
    std::ifstream trajectory_file("trajectory.txt");
    if (trajectory_file.is_open())
    {   int b = 0;
        cout<<"open the Lidar txt successfully!"<<endl;
        while (!trajectory_file.eof())
        {   // 读取时间戳、旋转四元数和平移向量
            double stamp;
            Eigen::Quaternionf q;
            Eigen::Vector3f t;
            string COL;
            int usless;
            trajectory_file >> COL >> usless >> t.x() >> t.y() >> t.z() >>  q.w() >> q.x() >> q.y() >> q.z() >> stamp;
            // 将数据保存到PoseWithStamp结构体中，并将其添加到trajectory向量中
            PoseWithStamp pose;
            pose.stamp = stamp;
            pose.rotation = q;
            pose.translation = t;
            pose.idx = b;
            b++;
            trajectory_.push_back(pose);
        }
        trajectory_file.close();
    }
    else
    {   // 如果无法打开轨迹文件，则输出错误信息并返回-1
        std::cout << "Unable to open trajectory file" << std::endl;
        return -1;
    }

    // 读取相机位姿文件
    vector<ImageWithPose> cameraPoseAndVal;
    std::vector<PoseWithStamp> camera_poses;
    std::ifstream camera_pose_file("camera_pose.txt");
    if (camera_pose_file.is_open())
    {
        int a =0;
         while (!camera_pose_file.eof())
     {  
        // 读取时间戳和相机位姿
        double stamp;
        string camera;
        Eigen::Matrix4f pose_mat;
        camera_pose_file >> stamp >> camera;
        for (int i = 0; i < 4; i++)
        {
            for (int j = 0; j < 4; j++)
            {
                camera_pose_file >> pose_mat(i, j);
            }
        }
        // 从位姿矩阵中提取旋转四元数和平移向量，并保存到PoseWithStamp结构体中
        Eigen::Matrix3f R = pose_mat.block<3,3>(0, 0);
        Eigen::Quaternionf q(R);
        Eigen::Vector3f t = pose_mat.block<3,1>(0, 3);
        PoseWithStamp pose;
        pose.stamp = stamp;
        pose.rotation = q;
        pose.translation = t;
        pose.idx = a;
        a++;
        camera_poses.push_back(pose);
        
      }
      
      camera_pose_file.close();
      for (std::vector<PoseWithStamp>::iterator iter = camera_poses.begin(); iter != camera_poses.end(); ++iter)
      {
        ImageWithPose ThisPose;
        ThisPose.pose=*iter;
        cameraPoseAndVal.push_back(ThisPose);

      }

      cout<<"get cameraPoseAndVal sucessfully!"<<endl;
    }  
   
    
    else
    {
    // 如果无法打开相机位姿文件，则输出错误信息并返回-1
    std::cout << "Unable to open camera pose file" << std::endl;
    return -1;
    }

    // 执行插值
    poseAndtime = pose_interpolation(cameraPoseAndVal,trajectory_);
    cout<<"fished running pose_interpolation!"<<endl;

    // 将插值后的相机位姿写入文件
    std::ofstream interpolated_pose_file("interpolated_camera_pose.txt");
    if (interpolated_pose_file.is_open())
    {
    // 遍历image_hashtable_中的所有图像序列和位姿，并将其写入文件
        
            for (const auto& image_pose : poseAndtime)
            {
                 if (image_pose.valid)
                {
                  interpolated_pose_file << std::fixed << std::setprecision(6)
                                        << image_pose.pose.stamp << " "
                                        << image_pose.pose.rotation.w() << " "
                                        << image_pose.pose.rotation.x() << " "
                                        << image_pose.pose.rotation.y() << " "
                                        << image_pose.pose.rotation.z() << " "
                                        << image_pose.pose.translation.x()<< " "
                                        << image_pose.pose.translation.y()<< " "
                                        << image_pose.pose.translation.z()<< " "
                                        <<std::endl;
                }
            }
        
        interpolated_pose_file.close();
    }
    else
    {
        std::cout << "Unable to open interpolated pose file" << std::endl;
        return -1;
    }

    return 0;
}
