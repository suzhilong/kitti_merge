#include "opencv2/opencv.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>

void get_pcd_files(const std::string &path, std::map<int, std::string> &files)
{
    //得到1个map，key=pcd编号,value=(pcd的)路径+文件名
    //这里先用简单的
    //std::cout << "-------------get_pcd_files---------------" << std::endl;//cout
    for(int pcd_num = 0; pcd_num < 4541; pcd_num++)
    {
        int file_key = pcd_num;
        std::string name;
        std::stringstream ss;
        ss << pcd_num; 
        ss >> name;
        std::string file_value = path + name + ".pcd";
        files[file_key] = file_value;
        
        // //cout
        // std::cout << "pcd_files[" << pcd_num << "]:\n"
        //             << "key:" << file_key << "  "
        //             << "value:" << files[file_key] <<std::endl;
    }
}

void get_cam0_pose(const std::string &path, std::map<int, double[12]> &poses)
{
    //std::cout << "-------------get_cam0_pose---------------" << std::endl;//cout
    //得到1个map，key=编号,value=对应编号cam0的pose
    std::ifstream fin(path, std::ios::in);
    char line[1024]={"0"};
    int pcd_num = 0;
    int stop_flag = 0;//控制读取行数
    bool flag = false;
    while (fin.getline(line,sizeof(line)))
    {
        //////////////
        //控制功能
        if (flag == true && stop_flag == 500) 
        {
            break;
        }
        /////////////////

        //std::cout << "line" << pcd_num <<":" << std::endl;//cout

        std::stringstream word(line);
        for (int r = 0; r < 12; r++)
        {
            std::string temp;
            word >> temp;
            // std::cout << "temp: " << temp;//cout
            // std::cout << "  ";//cout
            poses[pcd_num][r] = stod(temp);
            // char temp_out[20];//cout
            // sprintf(temp_out,"%.15f",poses[pcd_num][r]);//cout
            // cout << "poses["<<pcd_num<<"]["<<r<<"]: " << temp_out <<"\n";//cout
        }
        pcd_num++;

        stop_flag++;
    }
}

void get_trans_v_c0(Eigen::Affine3d &v_c, Eigen::Affine3d &c_v)
{
    //std::cout << "-------------get_trans_v_c0---------------" << std::endl;//cout
    //form calib_velo_cam.txt
    Eigen::Matrix3d R_v_c0;
    //R
    R_v_c0 << 7.967514e-03,-9.999679e-01,-8.462264e-04,
            -2.771053e-03,8.241710e-04,-9.999958e-01,
            9.999644e-01,7.969825e-03,-2.764397e-03;
    //T
    v_c.translation() = Eigen::Vector3d(-0.01377769,-0.05542117,-0.2918589);

    v_c.linear() = R_v_c0;
    c_v = v_c.inverse();

    // //cout
    // Eigen::Matrix4d mat_v_c,mat_c_v;
    // mat_v_c = v_c.matrix();
    // mat_c_v = c_v.matrix();
    // std::cout << "Matrix v to c:\n" << mat_v_c << "\n"
    //             << "Matrix c to v:\n" << mat_c_v << std::endl;
}

//get tranform from  cam0 idx_1 pose to idx_2 pose
void get_trans(int idx_1, int idx_2, Eigen::Affine3d &T, std::map<int, double[12]> &poses_map)
{
    //std::cout << "-------------get_trans---------------" << std::endl;//cout
//get trans:idx_1 to idx_2
    Eigen::Matrix3d poses_1,poses_2;
    Eigen::Affine3d p_1, p_2;
//p_1
    //R
    poses_1 <<  poses_map[idx_1][0], poses_map[idx_1][1], poses_map[idx_1][2],
                poses_map[idx_1][4], poses_map[idx_1][5], poses_map[idx_1][6],
                poses_map[idx_1][8], poses_map[idx_1][9], poses_map[idx_1][10];
    //T
    p_1.translation() = Eigen::Vector3d(poses_map[idx_1][3],
                                        poses_map[idx_1][7],
                                        poses_map[idx_1][11]);

    p_1.linear() = poses_1;

//p_2
    //R

    poses_2 <<  poses_map[idx_2][0], poses_map[idx_2][1], poses_map[idx_2][2],
                poses_map[idx_2][4], poses_map[idx_2][5], poses_map[idx_2][6],
                poses_map[idx_2][8], poses_map[idx_2][9], poses_map[idx_2][10];
    //T
    p_2.translation() = Eigen::Vector3d(poses_map[idx_2][3],
                                        poses_map[idx_2][7],
                                        poses_map[idx_2][11]);

    p_2.linear() = poses_2;
    
//1 to 2's trans
    T = p_2.inverse() * p_1;

    // //cout : use to debug
    // Eigen::Matrix4d p_1_mat = p_1.matrix();
    // std::cout << "Here is " << idx_1 << " Matrix p_1:\n" << p_1_mat << std::endl;
    // Eigen::Matrix4d p_2_mat = p_2.matrix();
    // std::cout << "Here is " << idx_2 << " Matrix p_2:\n" << p_2_mat << std::endl;
    // Eigen::Matrix4d T_mat = T.matrix();
    // std::cout << "Here is pose[" << idx_1 << "] to pose[" << idx_2 << "] Matrix T:\n" 
    //             << T_mat << std::endl;
}

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        std::cout << "please input with format: ./pcd_merge [pcd_path] [pose_path] [opt_path_filename] \n";
        return 0;
    }
    std::string pcd_path = argv[1];
    std::string pose_path = argv[2];
    std::string output_pcd_path_file = argv[3];//记得最后加上文件名../merged.pcd

//get pcd_map and poses_map
    std::map<int, std::string> pcd_map;
    get_pcd_files(pcd_path, pcd_map);
    std::map<int, double[12]> poses_map;
    get_cam0_pose(pose_path, poses_map);

//get trans of velo to cam0
    Eigen::Affine3d affine_v_c,affine_c_v;
    get_trans_v_c0(affine_v_c,affine_c_v);

//merge all pcds
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sum(new pcl::PointCloud<pcl::PointXYZRGB>);

    int base_index = -1;
    int count=0;
    int save_num = 0;//用来标识是否保存
    for ( auto pcd_iter = pcd_map.begin() ; pcd_iter != pcd_map.end(); pcd_iter++)
    {
        if (save_num % 50 == 0)
        {//控制合成的数量
            if(base_index==-1)
            {
                base_index=pcd_iter->first;
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::io::loadPCDFile(pcd_iter->second, *cloud_temp);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_base(new pcl::PointCloud<pcl::PointXYZRGB>);
                //两种办法：
                //1.把第一帧转到cam0位姿，之后所有转到cam0变换后不用在转回velo
                //2.第一帧不动，之后的帧转到cam0的第一帧姿态后，再转回velo姿态拼接
                //Eigen::Matrix4d trans_v_c0 = affine_v_c.matrix();//方法1
                //pcl::transformPointCloud(*cloud_temp, *cloud_base, trans_v_c0);//方法1
                //*cloud_sum = *cloud_base;//方法1
                *cloud_sum = *cloud_temp;//方法2
            }
            else
            {
                Eigen::Matrix4d trans_to_base;
                Eigen::Affine3d affine_to_base;
                get_trans(pcd_iter->first, base_index, affine_to_base, poses_map);
                //Eigen::Affine3d affine_v_c0_base = affine_to_base * affine_v_c;//方法1
                Eigen::Affine3d affine_v_c0_base_v = affine_c_v * affine_to_base * affine_v_c;//方法2
                //trans_to_base = affine_v_c0_base.matrix();//方法1
                trans_to_base = affine_v_c0_base_v.matrix();//方法2 
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_j(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::io::loadPCDFile(pcd_iter->second, *cloud_j);
                pcl::transformPointCloud(*cloud_j, *cloud_tmp, trans_to_base);
                *cloud_sum += *cloud_tmp;
            }
            std::cout<<"merge the "<<count<<" number. "<<endl;
            count++;
        }
        save_num += 1;
    }
    pcl::io::savePCDFileBinary(output_pcd_path_file , *cloud_sum);

    return 0;
}