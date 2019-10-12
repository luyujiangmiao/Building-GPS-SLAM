//
//  Config2.h
//  ORB_SLAM2
//
//  Created by vision on 2019/5/11.
//
//

#ifndef Config2_H
#define Config2_H

#include <stdio.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <vector>
#include <math.h>
#include"Config.h"
using namespace std;
namespace ORB_SLAM2 {

class Config2{
public:

   static Eigen::Matrix3d Quat2RotaMatx(double scale, double i,double j, double k);
   static Eigen::Quaterniond RotaMatx2Quat(Eigen::Matrix3d R);
   static Eigen::Vector3d RotaMatx2eule(Eigen::Matrix3d R);
    
    //角度转弧度
    inline static double a2r(double angle)
    {
        return (angle*M_PI/180);
    }
    
    //弧度转角度
    inline static double r2a(double radius)
    {
        return (radius*180/M_PI);
    }
    
    static cv::Point3f transPw2Pc1(cv::Point3f& Pw, cv::Mat Tc1w_sensor);
    static vector<double> cal_eyeline(const float yaw,const cv::Point2f cam_loc);
    static int Judge_insectpt2(cv::Point2d insectpt, cv::Point2d cam_loc, double yaw);
    
    struct Point2i_Comp {
        template<typename PtrType>
        bool operator()(PtrType pt1, PtrType pt2) const{
            if (pt1.x == pt2.x) return pt1.y > pt2.y;
            else return pt1.x > pt2.x;
        }
    };
    typedef std::set<cv::Point2i, Point2i_Comp> PointSet;
    static PointSet find_seenfacade(const cv::Point2f&, const float before_yaw, int fov, map<int,map<int,vector<cv::Point3f>>>& map_buildings);
    static vector<cv::Point3f> find_closest_facade(const cv::Point3f& P3d, map<int,map<int,vector<cv::Point3f>>>& map_buildings, PointSet all_seenindex);
    static vector<cv::Point3f> find_closest_facade2(const cv::Point3f& P3d, map<int,map<int,vector<cv::Point3f>>>& map_buildings, PointSet all_seenindex,float& dis_out);
};
}
#endif /* Config2_hpp */
