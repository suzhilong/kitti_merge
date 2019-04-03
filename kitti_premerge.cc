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
        std::cout << "please input with format: ./pcd_premerge [pcd_path] [pose_path] [opt_path] \n";
        return 0;
    }
    std::string pcd_path = argv[1];
    std::string pose_path = argv[2];
    std::string output_pcd_path = argv[3];//小规模合并点云输出地址/opt/premerged/

//get pcd_map and poses_map
    std::map<int, std::string> pcd_map;
    get_pcd_files(pcd_path, pcd_map);
    std::map<int, double[12]> poses_map;
    get_cam0_pose(pose_path, poses_map);

//get trans of velo to cam0
    Eigen::Affine3d affine_v_c,affine_c_v;
    get_trans_v_c0(affine_v_c,affine_c_v);

    long a = 0;//控制合成数量
    for ( auto pcd_iter = pcd_map.begin() ; pcd_iter != pcd_map.end(); pcd_iter++)
    {
    ////////////////////////////////////////////
        if(a<0)
        {
            a++;
            continue;
        }
                        ///////先拼一部分看效果
        if(a>10)
        {
            break;
        }
    ///////////////////////////////////////////
        std::cout << "merge the " << pcd_iter->first << " pcd." << std::endl;
        if(a>-1 && a<pcd_map.size()-10)
        {//后几个拼接到第一个
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_base(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::io::loadPCDFile(pcd_iter->second, *cloud_base);
            auto begin_iter=pcd_iter;
            auto end_iter=pcd_iter;
            for(int count = 0;count<10;count++)
            {//10个一组
                 //begin_iter-- ;
                 end_iter++ ;
            }
            end_iter++;
            for(auto j = begin_iter; j != end_iter; j++ )
            {//把后面pcd转换拼接到第1个上
                if(j==pcd_iter)
                {
                    continue;
                }
            //get cam0 pose tansform matrix
                Eigen::Affine3d affine_end_begin;
                get_trans(j->first, begin_iter->first, affine_end_begin, poses_map);

            //transform
                Eigen::Matrix4d trans_end_begin;
                trans_end_begin = (affine_c_v * affine_end_begin * affine_v_c).matrix();
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_j(new pcl::PointCloud<pcl::PointXYZRGB>);
                pcl::io::loadPCDFile(j->second, *cloud_j);
                pcl::transformPointCloud(*cloud_j, *cloud_tmp, trans_end_begin);
                *cloud_base += *cloud_tmp;
            }
            std::string name = std::to_string(pcd_iter->first);
            pcl::io::savePCDFileBinary(output_pcd_path + name + ".pcd", *cloud_base);
        }
        else
        {//直接保存
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::io::loadPCDFile(pcd_iter->second, *cloud_tmp);
            std::string name = std::to_string(pcd_iter->first);
            pcl::io::savePCDFileBinary(output_pcd_path + name + ".pcd", *cloud_tmp);
        }
        a++;
    }
    return 0;
}