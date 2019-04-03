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
    for(int pcd_num = 0; pcd_num < 4541; pcd_num++)
    {
        int file_key = pcd_num;
        std::string tem;
        std::stringstream ss;
        ss << pcd_num; 
        ss >> tem;
        int n = 6 - tem.size();
        std::string name = tem;
        for (int a=n; a > 0; a--)
        {
            name = "0" + name;
        }
        std::string file_value = "/home/su/code/code/kitti00/velodyne/00/pcds/" + name + ".pcd";
        files[file_key] = file_value;
    }
}

void get_img_files(const std::string &path, std::map<int, std::string> &files)
{
    //得到1个map，key=img编号,value=(img的)路径+文件名
    //这里先用简单的
    for(int img_num = 0; img_num < 4541; img_num++)
    {
        int file_key = img_num;
        std::string tem;
        std::stringstream ss;
        ss << img_num; 
        ss >> tem;
        int n = 6 - tem.size();
        std::string name = tem;
        for (int a=n; a > 0; a--)
        {
            name = "0" + name;
        }
        std::string file_value = "/home/su/code/code/kitti00/color/00/image_2/" + name + ".png";
        files[file_key] = file_value;
    }
}

std::vector<cv::Point3d> get3DPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    //从点云提取x,y,z坐标点
    std::vector<cv::Point3d> points;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *cloud_f);
    for (int i = 0; i < cloud->points.size(); i++)
    {
        points.push_back(cv::Point3d(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z));
    }

    return points;
}


int main(int argc, char **argv)
{
    if (argc != 5)
    {
        std::cout << "please input with format: ./color_1 [pcd_path] [img_path] [output_pcd_dir] [range limitation, 0 means no limitation]\n";
        return 0;
    }
//pcd_path
    std::string pcd_path = argv[1];//pcd的保存路径
//img_path
    std::string img_path = argv[2];//图片的保存路径
//colored_cloudpoint save path
    std::string output_pcd_path = argv[3];//上色后的点云保存地址../opt/pcd_colored
//limitation
    double range = std::stod(argv[4]);


//ext transform between lidar and camera
    //calib_velo_cam.txt
    Eigen::Matrix4d trans_v_c0, trans_c0_v;
    Eigen::Affine3d extrinsic_v_c0;
    Eigen::Matrix3d R_v_c0;
    //R
    R_v_c0 << 7.967514e-03,-9.999679e-01,-8.462264e-04,
            -2.771053e-03,8.241710e-04,-9.999958e-01,
            9.999644e-01,7.969825e-03,-2.764397e-03;
    //T
    extrinsic_v_c0.translation() = Eigen::Vector3d(-0.01377769,-0.05542117,-0.2918589);

    extrinsic_v_c0.linear() = R_v_c0;
    trans_v_c0 = extrinsic_v_c0.matrix();
    trans_c0_v = extrinsic_v_c0.inverse().matrix();
    std::cout << "trans_v_c0: \n"
                << trans_v_c0 << "\n"
                << "trans_c0_v: \n"
                << trans_c0_v << std::endl;
    
    //trans cam0 to cam2
    Eigen::Matrix4d trans_c0_c2, trans_c2_c0;
    Eigen::Affine3d extrinsic_c0_c2;
    Eigen::Matrix3d R_c0_c2;
    //R
    R_c0_c2 << 9.999788e-01,-5.008404e-03,-4.151018e-03,
                4.990516e-03,9.999783e-01,-4.308488e-03,
                4.172506e-03,4.287682e-03,9.999821e-01;
    //T
    extrinsic_c0_c2.translation() = Eigen::Vector3d(5.954406e-02,-7.675338e-04,3.582565e-03);

    extrinsic_c0_c2.linear() = R_c0_c2;
    trans_c0_c2 = extrinsic_c0_c2.matrix();
    trans_c2_c0 = extrinsic_c0_c2.inverse().matrix();
//trans_v_c2
    Eigen::Matrix4d trans_v_c2 = trans_c0_c2 * trans_v_c0;
    Eigen::Matrix4d trans_c2_v = trans_v_c2.inverse();

//intrisic transform of cam2
    // Intrisic matrix
    cv::Mat intrisicMat(3, 3, cv::DataType<double>::type);
    intrisicMat.at<double>(0, 0) = 7.188560000000e+02;//fcx
    intrisicMat.at<double>(1, 0) = 0;
    intrisicMat.at<double>(2, 0) = 0;

    intrisicMat.at<double>(0, 1) = 0;
    intrisicMat.at<double>(1, 1) = 7.188560000000e+02;//fcy
    intrisicMat.at<double>(2, 1) = 0;

    intrisicMat.at<double>(0, 2) = 6.071928000000e+02;//Cx
    intrisicMat.at<double>(1, 2) = 1.852157000000e+02;//Cy
    intrisicMat.at<double>(2, 2) = 1;0;
    std::cout << "intrisic matrix: \n"
            << intrisicMat << std::endl;
    
    cv::Mat rVec(3, 1, cv::DataType<double>::type); // Rotation vector
    rVec.at<double>(0) = 0;
    rVec.at<double>(1) = 0;
    rVec.at<double>(2) = 0;

    cv::Mat tVec(3, 1, cv::DataType<double>::type); // Translation vector
    tVec.at<double>(0) = 0;
    tVec.at<double>(1) = 0;
    tVec.at<double>(2) = 0;

    cv::Mat distCoeffs(5, 1, cv::DataType<double>::type); // Distortion vector
    distCoeffs.at<double>(0) = -3.685917e-01;
    distCoeffs.at<double>(1) = 1.928022e-01;
    distCoeffs.at<double>(2) = 4.069233e-04;
    distCoeffs.at<double>(3) = 7.247536e-04;
    distCoeffs.at<double>(4) = -6.276909e-02;

    std::map<int, std::string> pcd_map;//pcd路径
    std::map<int, std::string> img_map;//图片路径
    get_pcd_files(pcd_path, pcd_map);
    get_img_files(img_path, img_map);


    for(auto pcd_iter = pcd_map.begin(); pcd_iter != pcd_map.end(); pcd_iter++)
    {//一张一张pcd做
    //load pcd
        std::cout << "loading pcd: " << pcd_iter->second <<"\n";
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZ>);
        if(pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_iter->second, *cloud_src)==-1)
        {
            std::cout << "load pcd failed" << std::endl;
            continue;
        }
    //transform pcd pose to cam2 pose
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cam(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud_src, *cloud_cam, trans_v_c2);

    //get 3D points
        std::vector<cv::Point3d> objectPoints = get3DPoints(cloud_cam);

    //通过给定的内参数和外参数计算三维点投影到二维图像平面上的坐标
        std::vector<cv::Point2d> imagePoints;
        cv::projectPoints(objectPoints, rVec, tVec, intrisicMat, distCoeffs, imagePoints);

    //load matched img
        std::cout << "loading image: " << img_map[pcd_iter->first] <<"\n";
        cv::Mat image;
        image = cv::imread(img_map[pcd_iter->first], CV_LOAD_IMAGE_COLOR);

        cv::Mat depth = cv::Mat::zeros(image.cols, image.rows, CV_32FC1);//x向右，y向下，所以x是列，y是行

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cam_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);

        for (int i = 0; i < imagePoints.size(); i++)
        {//点云转2D图片，转换后的点(x,y)按照匹配的图片的(x,y)给点云上色
        //一个点一个点上
            auto col = round(imagePoints[i].x);//3D投影到2D的点
            auto row = round(imagePoints[i].y);
            auto x = cloud_cam->points[i].x;//照片位姿的点云
            auto y = cloud_cam->points[i].y;
            auto z = cloud_cam->points[i].z;

            if (z <= 0)
            {
                continue;
            }

            bool hide_flag = false;
            if (col >= 0 && col < image.cols && row >= 0 && row < image.rows)
            {
                if( image.at<cv::Vec3b>(row, col)[2]==0 && image.at<cv::Vec3b>(row, col)[1]==0 && image.at<cv::Vec3b>(row, col)[0]==0)
                {
                    continue;
                }

                if (depth.at<float>(row, col) == 0)
                {
                    depth.at<float>(row, col) = x * x + y * y + z * z;
                }
                else if (x * x + y * y + z * z < depth.at<float>(row, col))
                {
                    depth.at<float>(row, col) = x * x + y * y + z * z;
                    hide_flag = true;
                }
            }

            if (col >= 0 && col < image.cols && row >= 0 && row < image.rows && (!range || (x * x + y * y + z * z < range * range)) && !hide_flag)
            {//上色
                pcl::PointXYZRGB point;
                point.x = x;
                point.y = y;
                point.z = z;
                point.r = image.at<cv::Vec3b>(row, col)[2];
                point.g = image.at<cv::Vec3b>(row, col)[1];
                point.b = image.at<cv::Vec3b>(row, col)[0];
                cloud_cam_rgb->points.push_back(point);
            }
        }
        std::cout << cloud_cam_rgb->points.size() <<" can be used "<<"\n";

        //上色完后转回雷达位姿的pcd
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_velo_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::transformPointCloud(*cloud_cam_rgb, *cloud_velo_rgb, trans_c2_v);

        if (cloud_velo_rgb->points.size() != 0)
        {//保存上色后的点云
            cloud_velo_rgb->width = 1;
            cloud_velo_rgb->height = cloud_velo_rgb->points.size();
            // std::cout<<pcd_iter->second;
            std::string name = std::to_string(pcd_iter->first);
            std::cout << output_pcd_path << name << ".pcd" << std::endl;
            pcl::io::savePCDFileBinary(output_pcd_path + name + ".pcd", *cloud_velo_rgb);
            std::cout << "Saved " << cloud_velo_rgb->points.size() << " data points to " << name << ".pcd." << std::endl;
        }
    }
    return 0;
}