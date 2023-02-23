#include<iostream>
#include<string>
#include<Eigen/Core>
#include<fstream>
#include<vector>
#include <unordered_map>       // 使用无序哈希表实现图像序列的存储
#include <algorithm>           // 使用sort函数排序
#include <iomanip>             // 使用iomanip库输出浮点数精度
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

using namespace std;

struct TranslationWithTime {
        double time;
        double x,y,z;
};
struct xyzscale{
    double x_scale,y_scale,z_scale;
};

vector<TranslationWithTime> FindSameTimeInLidar(vector<TranslationWithTime> Camera_Translation,vector<TranslationWithTime> Lidar_Translation)
{   
    
    vector<TranslationWithTime> sametimeLidar;
    TranslationWithTime One_camera;
    double one_CameraTime;
    TranslationWithTime One_Lidar;
    double one_LidarTime;
    while (!Camera_Translation.empty() && !Lidar_Translation.empty())
    {
        
        One_camera = Camera_Translation.back();
        one_CameraTime = One_camera.time;
        One_Lidar = Lidar_Translation.back();
        one_LidarTime = One_Lidar.time;
        if (fabs(one_LidarTime - one_CameraTime)<0.001)
        {
            
            TranslationWithTime one_timeLidar;
            one_timeLidar.time = one_LidarTime;
            one_timeLidar.x = One_Lidar.x;
            one_timeLidar.y = One_Lidar.y;
            one_timeLidar.z = One_Lidar.z;
            sametimeLidar.push_back(one_timeLidar);
            Camera_Translation.pop_back();
            
            Lidar_Translation.pop_back();
        }
        else
        {
            Lidar_Translation.pop_back();
            
        }        
    }
    std::reverse(sametimeLidar.begin(),sametimeLidar.end());
    return sametimeLidar;
}

    // for(vector<TranslationWithTime>::iterator Cit = Camera_Translation.begin();Cit != Camera_Translation.end();++Cit)
    // {   
    //     One_camera = *Cit;
    //     one_CameraTime = One_camera.time;
    //     for(vector<TranslationWithTime>::iterator Lit = Lidar_Translation.begin();Lit != Lidar_Translation.end();++Lit)
    //     {
    //         One_Lidar = *Lit;
    //         one_LidarTime = One_Lidar.time;
    //         if (one_CameraTime==one_LidarTime)
    //         {

    //             break;
    //         }
    //         else
    //         {
                
    //         }
            

    //     } 
    // }


double computescale(vector<TranslationWithTime>& Camera_Translation,vector<TranslationWithTime>& SameTimeLidar_Translation)
{

    xyzscale scale;
    vector<xyzscale> scales;
    vector<std::pair<TranslationWithTime,TranslationWithTime>> CameraAndLidar;
    if (Camera_Translation.size()!=SameTimeLidar_Translation.size())
    {
        cout<<"error! "<<"Camera has "<<Camera_Translation.size()<<" data. but Lidar have "<<SameTimeLidar_Translation.size()<<" data"<<endl;
        return -1;
    }
    
    while (!Camera_Translation.empty())
   {
        TranslationWithTime C,L;
        C=Camera_Translation.back();
        L=SameTimeLidar_Translation.back();
        if (fabs(C.time-L.time)>0.01)
        {
            cout<<"time is not same"<<endl;
            return -1;
        }

        CameraAndLidar.emplace_back(C,L);
        Camera_Translation.pop_back();
        SameTimeLidar_Translation.pop_back();
    }

    std::pair<TranslationWithTime,TranslationWithTime> last_data,front_data;
    while (!CameraAndLidar.empty())
    {
    last_data = CameraAndLidar.back();
    CameraAndLidar.pop_back();
    front_data = CameraAndLidar.back();
    
    double x_camera_interpolation = last_data.first.x-front_data.first.x;
    double y_camera_interpolation = last_data.first.y-front_data.first.y;
    double z_camera_interpolation = last_data.first.z-front_data.first.z;

    double x_lidar_interpolation = last_data.second.x-front_data.second.x;
    double y_lidar_interpolation = last_data.second.y-front_data.second.y;
    double z_lidar_interpolation = last_data.second.z-front_data.second.z;

    scale.x_scale= x_camera_interpolation/x_lidar_interpolation;
    scale.y_scale = y_camera_interpolation/y_lidar_interpolation;
    scale.z_scale = z_camera_interpolation/z_lidar_interpolation;

    scales.push_back(scale);

    }
    ofstream scalefile("scale.txt");
    if (scalefile.is_open())
    {
        cout<<"begin write the scale"<<endl;
        for (int i = 0; i < scales.size(); i++)
        {
            scalefile << scales[i].x_scale<<" " <<scales[i].y_scale<<" "<<scales[i].z_scale<<endl;
        }
        scalefile.close();
        cout<<"write successfully"<<endl;
        
    }
    else{
        cout<<"can't open the scale.txt"<<endl;
        return -1;
    }
    double all_scale =0;
    for (vector<xyzscale>::iterator it= scales.begin(); it!=scales.end(); ++it)
    {
        all_scale +=it->x_scale;
        all_scale +=it->y_scale;
        all_scale +=it->z_scale;
        
    }
    return all_scale/(scales.size()*3);
}

int main()
{
    std::vector<TranslationWithTime> Camera_TimeAndTranslation;
    std::fstream Camera_file("interpolated_camera_pose.txt");
    if(Camera_file.is_open())
    {
        cout<<"open camera_file sucessfully!"<<endl;
        while (!Camera_file.eof())
        {                        
        double stamp;
        double tran_x,tran_y,tran_z;
        double usless;
        Camera_file >> stamp >> usless >> usless >> usless >> usless >> tran_x>>tran_y>>tran_z;
        TranslationWithTime T;
        T.time = stamp;
        T.x = tran_x;
        T.y = tran_y;
        T.z = tran_z;
        Camera_TimeAndTranslation.push_back(T);
        }
        Camera_file.close();
        Camera_TimeAndTranslation.pop_back();
    }
    else
    {
        cout<<"can't open camera_file"<<endl;
        return -1;
    }
    std::vector<TranslationWithTime> Lidar_TimeAndTranslation;
    std::fstream Lidar_file("trajectory.txt");

    if (Lidar_file.is_open())
    {
        
        cout<<"open the Lidar file sucessfully!"<<endl;
        while (!Lidar_file.eof())
        {   
            double L_stamp;
            double L_x,L_y,L_z;
            std::string usless_1;
            double usless_2;
            Lidar_file >> usless_1 >> usless_2 >> L_x >> L_y >> L_z >> usless_2 >> usless_2 >> usless_2 >> usless_2 >> L_stamp;
            TranslationWithTime L;
            L.time = L_stamp;
            L.x = L_x;
            L.y = L_y;
            L.z = L_z;
            Lidar_TimeAndTranslation.push_back(L);
            
            
        }
        Lidar_file.close();
        
    }
    else
    {
        cout<<"can't open the Lidar file"<<endl;
        return -1;
    }    
    
    vector<TranslationWithTime> TheSameTimeLidar = FindSameTimeInLidar(Camera_TimeAndTranslation,Lidar_TimeAndTranslation);
    double average_scale = computescale(Camera_TimeAndTranslation,TheSameTimeLidar);
    cout<<"the average scale is "<<average_scale<<endl;
    return 0;


}