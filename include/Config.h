
#ifndef Config_H
#define Config_H

#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pangolin/pangolin.h>
#include <math.h>

#include"Config2.h"
#include"Read2_5Dmap.h"
using namespace std;
namespace ORB_SLAM2 {

class Config{
public:
    
    static double eps;
    static int sgn(double x);
    static cv::Point2i Fn_Projection_pt(cv::Point3f points3f, cv::Mat Tcw);
    static cv::Mat Fn_ProjectionAR(cv::Mat, int, cv::Mat, vector<cv::Point3f> facade, cv::Mat ca_Tcw);
    

    static cv::Point3f vec2pt3d(const vector<double>);
    
    static double PointToSegDist(double x, double y, double x1, double y1, double x2, double y2);//point-segline
    static double cal_distance2(cv::Point2f, cv::Point2f);//point-point;

    static void mergeImg(cv::Mat &dst, cv::Mat src1, cv::Mat src2);
    
    static map<int,map<int,vector<cv::Point3f>>> cal_2dbuilding(const string &, const cv::Mat&);
    
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

    
    static void cal_cross(const cv::Point3f u, const cv::Point3f v, cv::Point3f& cross);

    //2点计算直线
    static vector<double> cal_buildingline(const cv::Point2d&eyeline, const cv::Point2d& building);


    //求交点
    static cv::Point2d cal_insetpt(const vector<double>&,const vector<double>&);

    //判断交点1:是不是建筑物线上
    static int Judge_insectpt1( cv::Point2d insectpt, cv::Point2d pt1, cv::Point2d pt2);

    
};

}
#endif
