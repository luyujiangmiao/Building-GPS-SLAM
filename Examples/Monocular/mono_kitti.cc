/**
 * This file is part of ORB-SLAM2.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * ORB-SLAM2 is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
 */


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<string.h>
#include <unistd.h>
#include<opencv2/core/core.hpp>
#include <future>
#include<thread>

#include"System.h"

#define PI 3.1415926
#include <math.h>

#include"Read2_5Dmap.h"
#include"Config.h"
#include "Config2.h"

#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp> //realted to cv::mat <-> eigen

using namespace std;
int Sign(double test);

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames, vector<string> &vstrDepthMasknames,
                vector<string>& vstrDepthtxtnames, vector<double> &vTimestamps, vector<cv::Mat> &sensor_poses,
                vector<float>& rela_Euleryaw, cv::Mat& Twc1);
void replace_string(std::string &strBig, const std::string &strsrc, const std::string &strdst);



int processing(char **argv, ORB_SLAM2::System *slamPtr);
int main(int argc, char **argv)
{
    ofstream NULL_FILE("/dev/null");
    
    //std::cout.rdbuf(NULL_FILE.rdbuf());
    //std::cerr.rdbuf(NULL_FILE.rdbuf());
    //    if(argc != 4)
    //    {
    //        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
    //        return 1;
    //    }
    //liuliu
    //Global
    string strVocFile = "/Users/vision/Downloads/luyu/project/ORB_SLAM_xcode_graz/Vocabulary/ORBvoc.txt";
    string strSettingFile = "/Users/vision/Downloads/luyu/project/ORB_SLAM_xcode_graz/Examples/Monocular/Surfacecam.yaml";
    string strPathToSequence = "/Users/vision/Downloads/luyu/buildingdataset/ismar_github/07-08/";
    string mapfilepath = "/Users/vision/Downloads/luyu/buildingdataset/ismar_github/gis.json";
    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<string> vstrDepthMasknames;
    vector<string> vstrDepthtxtnames;
    vector<double> vTimestamps;
    
    //addby liuliu
    vector<cv::Mat> sensor_poses;
    vector<float> rela_Euleryaw;
    cv::Mat Twc1_formap;
    
    //    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);
    LoadImages(strPathToSequence, vstrImageFilenames, vstrDepthMasknames, vstrDepthtxtnames, vTimestamps, sensor_poses, rela_Euleryaw, Twc1_formap);
   
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    //    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
    ORB_SLAM2::System SLAM(strVocFile, vstrImageFilenames[0], strSettingFile, mapfilepath, Twc1_formap, sensor_poses[0], ORB_SLAM2::System::MONOCULAR,true);
    
    int nImages = vstrImageFilenames.size();
    
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
    
    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;
    
    // Main loop
    int main_error = 0;
    std::thread main_loop([&](){
        cv::Mat im,imDepthmask;
        for(int ni=0; ni<nImages; ni++)
        {
            // Read image from file
            im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
            imDepthmask = cv::imread(vstrDepthMasknames[ni],CV_LOAD_IMAGE_UNCHANGED);

            double tframe = vTimestamps[ni];
            cv::Mat T_sensor(4,4,CV_64F);
            T_sensor = sensor_poses[ni];
            float rela_yaw;
            rela_yaw = rela_Euleryaw[ni];
            if(im.empty())
            {
                cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
                main_error = 1;
                return;
            }
            
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            
            // Pass the image to the SLAM system
            SLAM.TrackMonocular(im,imDepthmask, vstrImageFilenames[ni], vstrDepthMasknames[ni], vstrDepthtxtnames[ni], tframe, T_sensor, rela_yaw);
            
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

            
            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            
            vTimesTrack[ni]=ttrack;
            
            // Wait to load the next frame
            double T=0;
            if(ni<nImages-1)
                T = vTimestamps[ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestamps[ni-1];
            
            T = T *1e-7;
            if(ttrack<T)
                usleep((T-ttrack)*1e6);
        }
        SLAM.Shutdown();

    });
    SLAM.StartViewer();
    
    main_loop.join();
    if(main_error != 0) return main_error;
    
    // Stop all threads
    
    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    
    // Save camera trajectory
    SLAM.SaveMapPoint("../results/07-08/MapPoints.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("../results/07-08/KeyFrameTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM_sensor("../results/07-08/KeyFrameTrajectory_sensor.txt");
    return 0;
}

int Sign(double test)
{
    if(test>0) return 1;
    else if(test<0) return -1;
}


void replace_string( std::string &strBig, const std::string &strsrc, const std::string &strdst)
{
    std::string::size_type pos = 0;
    std::string::size_type srclen = strsrc.size();
    std::string::size_type dstlen = strdst.size();
    while( (pos=strBig.find(strsrc, pos)) != std::string::npos )
    {
        strBig.replace( pos, srclen, strdst );
        pos += dstlen;
    }
    
}
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<string> &vstrDepthMasknames,
                vector<string>& vstrDepthtxtnames, vector<double> &vTimestamps, vector<cv::Mat> &sensor_poses, vector<float>& rela_Euleryaw, cv::Mat& Twc1_sensor)
{
    int count=1;
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/poses.txt";
    vector<string> timestamps;
    vector<cv::Mat> poss;
    vector<cv::Mat> Qs;
    float first_euleryaw;
    
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        int start_image_num = 1;
        int end_image_num = 10000;
        string s;
        getline(fTimes,s);
        string timestamp;
        //double x,y,z;
        cv::Mat pos = cv::Mat(3, 1, CV_64F);
        stringstream x_,y_,z_;
        stringstream q1_,q2_,q3_,q4_;
        //liuliu
        if(count > start_image_num &&  count < end_image_num){

            if(!s.empty())
            {
        
                 
                 /******************timestamps***********/
                timestamp = s.substr(21, 10);
                stringstream ss;
                ss << timestamp;
                double t;
                ss >> t;
                vTimestamps.push_back(t);
                timestamps.push_back(timestamp);
                /******************pos read*******************/
                char* pt_x = strstr(&s[0],"x");
                int pt_xi = pt_x - &s[0];
                char* pt_y = strstr(&s[0],"y");
                int pt_yi = pt_y - &s[0];;
                char* pt_alt = strstr(&s[0],"alt");
                int pt_alti = pt_alt - &s[0];

                x_ << s.substr(pt_xi+3, 11);
                y_ << s.substr(pt_yi+3, 9);
                z_ << s.substr(pt_alti+5, 11);// has 18 sites
                x_ >> pos.at<double>(0,0);
                y_ >> pos.at<double>(0,1);
                z_ >> pos.at<double>(0,2);
                
//                cout<<"pos: "<<pos<<endl;
                pos.at<double>(0,0) = (pos.at<double>(0,0) - 534796.31);
                pos.at<double>(0,1) = (pos.at<double>(0,1) - 5211807.79);
                if( pos.at<double>(0,2) >0)
                {
                    pos.at<double>(0,2) = (pos.at<double>(0,2) - 356.5);
                }
                poss.push_back(pos);
                /************* orientation read***************/
                int q_site = 15;
                char* pt_orien = strstr(&s[0],"orientation");
                int pt_orieni = pt_orien - &s[0];
                char* pt_q2 = strstr(&s[pt_orieni],"y");
                int pt_q2i = pt_q2 - &s[pt_orieni];
                char* pt_q3 = strstr(&s[pt_orieni],"z");
                int pt_q3i = pt_q3 - &s[pt_orieni];
                char* pt_q4 = strstr(&s[pt_orieni],"w");
                int pt_q4i = pt_q4 - &s[pt_orieni];
                
                q1_ << s.substr(pt_orieni+18, q_site);
                q2_ <<  s.substr(pt_orieni+pt_q2i+3, q_site);
                q3_ << s.substr(pt_orieni+pt_q3i+3, q_site);
                q4_ << s.substr(pt_orieni+pt_q4i+3, q_site);
                double temp_q[4];
                q1_ >> temp_q[0];
                q2_ >> temp_q[1];
                q3_ >> temp_q[2];
                q4_ >> temp_q[3];
                cv::Mat Q = cv::Mat(4, 1, CV_64F, temp_q);
                Eigen::Matrix3d Ruc_sensor_file, DetlaRwor2unity, Rwc_sensor;
                Ruc_sensor_file = ORB_SLAM2::Config2::Quat2RotaMatx(temp_q[0], temp_q[1], temp_q[2], temp_q[3]);
                DetlaRwor2unity <<1,0,0,
                                  0,0,-1,
                                  0,1,0;
                Rwc_sensor = DetlaRwor2unity*Ruc_sensor_file;
                
                Eigen::Matrix3d Rcw_sensor;
                Rcw_sensor = Rwc_sensor.inverse();
                Eigen::Vector3d angle_E;
                angle_E = ORB_SLAM2::Config2::RotaMatx2eule(Rcw_sensor);
                cv::Point3f angle;
                angle.x = angle_E(0);
                angle.y = angle_E(1);
                angle.z = angle_E(2);
                angle.x = angle.x*180.0/M_PI;
                angle.y = angle.y*180.0/M_PI;
                angle.z = angle.z*180.0/M_PI;
                
                /*******************T************************/
                cv::Mat Twc_sensor = cv::Mat::eye(4, 4, CV_64F);
                cv::Mat Rwc_sensor_mat = cv::Mat(3, 3, CV_64F);
                eigen2cv(Rwc_sensor, Rwc_sensor_mat);
                
                Rwc_sensor_mat.copyTo(Twc_sensor.rowRange(0,3).colRange(0,3));
                pos.copyTo(Twc_sensor.rowRange(0,3).col(3));
                
                //record the first sensor pose
                if(count == start_image_num+1)
                {
                    Twc1_sensor = Twc_sensor;
                    first_euleryaw = angle.z;
                }
                angle.z = angle.z-first_euleryaw;
                rela_Euleryaw.push_back(angle.z);
                sensor_poses.push_back(Twc_sensor);

            }
        }
        count++;
    }
    
    string strPrefixLeft = strPathToSequence + "63676070";
    
    
    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);
    vstrDepthMasknames.resize(nTimes);
    vstrDepthtxtnames.resize(nTimes);
    
    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        string suffix = "depth_63676070";//07-08

        vstrImageFilenames[i] = strPrefixLeft + timestamps[i] + ".png";
        vstrDepthMasknames[i] = vstrImageFilenames[i];
        replace_string( vstrDepthMasknames[i], "63676070", suffix);
        vstrDepthtxtnames[i] = vstrDepthMasknames[i];
        replace_string(vstrDepthtxtnames[i],"png","txt");

    }
}
