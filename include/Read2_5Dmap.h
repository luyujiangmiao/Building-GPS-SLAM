#ifndef Read2_5Dmap_H
#define Read2_5Dmap_H


#include<iostream>

//for reade json
#include<fstream>
#include<cstring>
#include "jsonxx.h"

#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<map>

#include"Config.h"
#include"Config2.h"
using namespace std;
using namespace cv;
//
class Read2_5map
{
public:
    Read2_5map(const string& strjsonfilePath);

    void LoadObjData(const string& sobj, vector<cv::Point3d>&,
                       vector<cv::Point3d>&,  vector<vector<cv::Point3i>>&,
                     /*vector<cv::Point2d>&, vector<vector<cv::Point3i>>&,*/ int&);

    void Parse_json(const string& strjsonfilePath);

    void save_obj(string,const cv::Mat& Tc1w_formap);
    map<int, vector<cv::Point3d>> map_vertics;
    map<int, vector<cv::Point3d>> map_normals;
    map<int, vector<vector<cv::Point3i>>> map_faceData;

    //2d map
//    map<int, vector<cv::Point2d>> _2dmap_vertics;
//    map<int, vector<vector<cv::Point3i>>> _2dmap_faceData;
    map<int, int> facadesnum_eachbuilding;
    float scale;
    int json_size;

};
#endif
