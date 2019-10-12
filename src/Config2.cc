//
//  Config2.cpp
//  ORB_SLAM2
//
//  Created by Ruyu on 2019/5/11.
// Some basic function
//

#include "Config2.h"
namespace ORB_SLAM2{
    
    Eigen::Matrix3d Config2::Quat2RotaMatx(double scale, double i,double j, double k)
    {
        /*
         Convert quaternion to rotation matrix
         w is the scale in quaternion
         */
        Eigen::Quaterniond q;
        q.w() = scale;
        q.x() = i;
        q.y() = j;
        q.z() = k;
        q = q.normalized();
        Eigen::Matrix3d R = q.toRotationMatrix();
        return R;
    }
    
    Eigen::Quaterniond Config2::RotaMatx2Quat(Eigen::Matrix3d R)
    {/*
      Convert rotation matrix to quaternion
      w is the scale in quaternion
      */
        Eigen::Quaterniond q = Eigen::Quaterniond(R);
        q.normalize();
        return q;
    }
    Eigen::Vector3d Config2::RotaMatx2eule(Eigen::Matrix3d R)
    {/*
      Convert rotation matrix to euler angle
      */
        Eigen::Matrix3d m;
        m = R;
        Eigen::Vector3d euler = m.eulerAngles(0, 1, 2);

        return euler;
    }
    
    cv::Point3f Config2::transPw2Pc1(cv::Point3f& Pw, cv::Mat Tc1w_sensor)
    {/*
      Transform the 2.5D map in world coordinate rotation to the first sensor coordiate system
      */
        Tc1w_sensor.convertTo(Tc1w_sensor, CV_32F);
        cv::Mat Rc1w_sensor(3,3,CV_32F),tc1w_sensor(3,1,CV_32F);
        Rc1w_sensor = Tc1w_sensor.rowRange(0,3).colRange(0,3);
        tc1w_sensor = Tc1w_sensor.rowRange(0,3).col(3);
        cv::Mat v_(3,1,CV_32F), v1_(3,1,CV_32F);
        v_ = v_.t();
        v1_ = v1_.t();
        v_.at<float>(0) = Pw.x;
        v_.at<float>(1) = Pw.y;
        v_.at<float>(2) = Pw.z;
        v1_.at<float>(0) = Rc1w_sensor.row(0).dot(v_)+tc1w_sensor.at<float>(0);
        v1_.at<float>(1) = Rc1w_sensor.row(1).dot(v_)+tc1w_sensor.at<float>(1);
        v1_.at<float>(2) = Rc1w_sensor.row(2).dot(v_)+tc1w_sensor.at<float>(2);
        
        cv::Point3d Pc1;
        Pc1.x = v1_.at<float>(0);
        Pc1.y = v1_.at<float>(1);
        Pc1.z = v1_.at<float>(2);
        return Pc1;
        
    }
    
    vector<double> Config2::cal_eyeline(const float yaw,const cv::Point2f cam_loc)
    {
        /*
         cal the eyeline using yaw ang camera location
         0,180,360 no k
         90,270 can as a edge-case
         */
        vector<double> line;
        double temp_k, temp_b, temp_x0;
        float target_yaw;
        target_yaw = yaw;
        temp_k = 100000;
        temp_b = 100000;
        temp_x0 = 100000;
        
        if(target_yaw>=0)
        {
            if(target_yaw<=90)
                temp_k = tan(a2r(90-target_yaw));
            else if(target_yaw >90 && target_yaw<180)
                temp_k = -tan(a2r(target_yaw-90));
            else if (target_yaw>180 && target_yaw <=270)
                temp_k = tan(a2r(270-target_yaw));
            else if (target_yaw>270 && target_yaw<360)
                temp_k = -tan(a2r(target_yaw-270));
            else{
                temp_x0 = cam_loc.x;
                cout<<"no k"<<endl;
            }
        }
        
        else
        {
            target_yaw = -target_yaw;
            if(target_yaw<=90)
                temp_k = -tan(a2r(90-target_yaw));
            else if(target_yaw >90 && target_yaw<180)
                temp_k = tan(a2r(target_yaw-90));
            else if (target_yaw>180 && target_yaw <=270)
                temp_k = -tan(a2r(270-target_yaw));
            else if (target_yaw>270 && target_yaw<360)
                temp_k = tan(a2r(target_yaw-270));
            else{
                temp_x0 = cam_loc.x;
                cout<<"no k"<<endl;
            }
            
        }
        
        
        temp_b = cam_loc.y - temp_k*cam_loc.x;//b=y-kx;
        line.push_back(temp_k);
        line.push_back(temp_b);
        line.push_back(temp_x0);
        return line;
        
        
    }
    
    int Config2::Judge_insectpt2(cv::Point2d insectpt, cv::Point2d cam_loc, double yaw)
    {/*Judging the intersection point in the line of sight direction */
        if(yaw>360) yaw = yaw-360;
        
        
        if(yaw>=0)
        {
            if(fabs(yaw)<90)
            {
                if(insectpt.x>cam_loc.x && insectpt.y>cam_loc.y)
                    return 1;
                else
                    return 0;
            }
            else if(fabs(yaw) >=90 && fabs(yaw)<180)
            {
                if(insectpt.x>cam_loc.x && insectpt.y<cam_loc.y)
                    return 1;
                else
                    return 0;
            }
            else if (fabs(yaw)>=180 && fabs(yaw) <270)
            {
                if(insectpt.x<cam_loc.x && insectpt.y<cam_loc.y)
                    return 1;
                else
                    return 0;
            }
            else if (fabs(yaw)>=270 && fabs(yaw)<360)
            {
                if(insectpt.x<cam_loc.x && insectpt.y>cam_loc.y)
                    return 1;
                else
                    return 0;
            }
        }
        else
        {
            
            if(fabs(yaw)<90)
            {
                if(insectpt.x<cam_loc.x && insectpt.y>cam_loc.y)
                    return 1;
                else
                    return 0;
            }
            else if(fabs(yaw) >=90 && fabs(yaw)<180)
            {
                if(insectpt.x<cam_loc.x && insectpt.y<cam_loc.y)
                    return 1;
                else
                    return 0;
            }
            else if (fabs(yaw)>=180 && fabs(yaw) <270)
            {
                if(insectpt.x>cam_loc.x && insectpt.y<cam_loc.y)
                    return 1;
                else
                    return 0;
            }
            else if (fabs(yaw)>=270 &&fabs(yaw)<360)
            {
                if(insectpt.x>cam_loc.x && insectpt.y>cam_loc.y)
                    return 1;
                else
                    return 0;
            }
            
        }
    }
    
    Config2::PointSet Config2::find_seenfacade(const cv::Point2f&camera_pt, const float before_yaw, int fov, map<int,map<int,vector<cv::Point3f>>>& map_buildings)
    {/*find all seenfacede in 2d map using current camera yaw and location
      return the index of seenfacade*/
        cv::Mat showimg(600,600,CV_8UC3,cv::Scalar(255,255,255));
        cv::circle(showimg, cv::Point2i(camera_pt.x*1000+300,camera_pt.y*1000+300), 10, cv::Scalar(255,0,0));
        float before_yaw_fov;
        cv::Point2i mark_seenindex;//当前视线交点可见的面
        PointSet all_seenidex;//所有的可见建筑物下标

        vector<double> eyeline;
        for(int delta = -fov/2;delta <fov/2; delta=delta+3)//视角内的视线
        {
            before_yaw_fov = before_yaw+delta;
            eyeline = Config2::cal_eyeline(before_yaw_fov, camera_pt);
            double dis_of_thiseyeline = 100000;//80m 范围可见
            cv::Point2d mindis_pt;
            map<int,vector<cv::Point3f>> eachbuilding;
            for(int i=0;i<map_buildings.size();i++)// 循环每栋建筑物
            {
                eachbuilding = map_buildings[i];
                vector<cv::Point3f> facade_equation;
                for(int j=0;j<eachbuilding.size();j++)//循环每栋建筑物里的所有面
                {
                    facade_equation = eachbuilding[j];
                    vector<double> facade_line;
                    facade_line.push_back(facade_equation[2].x);
                    facade_line.push_back(facade_equation[2].y) ;
                    facade_line.push_back(facade_equation[2].z) ;
                    //视线和建筑线的交点
                    cv::Point2d insetpt;
                    insetpt = Config::cal_insetpt(eyeline, facade_line);
                    cv::Point3f pt1_ground,pt2_ground;
                    pt1_ground = facade_equation[0];
                    pt2_ground = facade_equation[1];
                    
                    cv::line(showimg, cv::Point2i(pt1_ground.x*1000+300,pt1_ground.z*1000+300), cv::Point2i(pt2_ground.x*1000+300,pt2_ground.z*1000+300), cv::Scalar(0,0,0));
             
                    int result1 = Config::Judge_insectpt1(insetpt, cv::Point2d(facade_equation[0].x, facade_equation[0].z), cv::Point2d(facade_equation[1].x,facade_equation[1].z));//交点是否在建筑物线段上
                    int result2 = Config2::Judge_insectpt2(insetpt, camera_pt, before_yaw_fov);//交点是否在视线方向上
                    
                    if( result1 && result2)
                    {

                        double temp_dis;
                        temp_dis = Config::cal_distance2(camera_pt, insetpt);

                        if(temp_dis<dis_of_thiseyeline)
                        {
                            
                            dis_of_thiseyeline = temp_dis;
                            mindis_pt = insetpt;
                            mark_seenindex.x = i;
                            mark_seenindex.y = j;

                        }
                    }
                }
                
            }//当前视线循环完地图
            //去除[0][0]
            if(dis_of_thiseyeline == 100000)
            {
                continue;
            }
            
            all_seenidex.insert(mark_seenindex);
//            cout<<"可见建筑物为：["<<mark_seenindex.x<<"]["<<mark_seenindex.y<<"] "<<endl;
            ////-------------for show
            cv::line(showimg, cv::Point2i(map_buildings[mark_seenindex.x][mark_seenindex.y][0].x*1000+300,map_buildings[mark_seenindex.x][mark_seenindex.y][0].z*1000+300), cv::Point2i(map_buildings[mark_seenindex.x][mark_seenindex.y][1].x*1000+300,map_buildings[mark_seenindex.x][mark_seenindex.y][1].z*1000+300), cv::Scalar(255,0,255),1);//红色的当前最近建筑物平面
            cv::circle(showimg, cv::Point2i(mindis_pt.x*1000+300,mindis_pt.y*1000+300), 2, Scalar(0,0,255));
            cv::line(showimg, cv::Point2i(camera_pt.x*1000+300,camera_pt.y*1000+300), cv::Point2i(mindis_pt.x*1000+300,mindis_pt.y*1000+300), cv::Scalar(255,255,0),1);//蓝色当前视线
//            cv::imshow("imshow", showimg);
//            cv::waitKey(0);
            cv::line(showimg, cv::Point2i(camera_pt.x*1000+300,camera_pt.y*1000+300), cv::Point2i(mindis_pt.x*1000+300,mindis_pt.y*1000+300), cv::Scalar(0,0,0),1);//黑色的视线
       
            ////---------------------

        }
        return all_seenidex;
    }
    
    
    vector<cv::Point3f> Config2::find_closest_facade(const cv::Point3f& P3d, map<int,map<int,vector<cv::Point3f>>>& map_buildings, Config2::PointSet all_seenindex)
    {
        map<int, vector<cv::Point3f>> cloest_facade_;
        vector<cv::Point3f> cloest_facade;
        double min_distance = 100000;
        cv::Point2i record_closet_buildnum;
        for (ORB_SLAM2::Config2::PointSet::iterator it=all_seenindex.begin(); it!=all_seenindex.end(); ++it)
        {
            cv::Point2i seen;
            seen = *it;
            vector<cv::Point3f> eachseenfacade = map_buildings[seen.x][seen.y];
            cv::Point3f normalvector_of_facade, pt_on_facade;
            pt_on_facade = eachseenfacade[4];
            normalvector_of_facade = eachseenfacade[5];
            
            double dis;
            cv::Point2f buildline_centerpt;
            buildline_centerpt.x = (eachseenfacade[0].x+eachseenfacade[1].x)/2;
            buildline_centerpt.y = (eachseenfacade[0].y+eachseenfacade[1].y)/2;
//            dis = Config::cal_distance2(cv::Point2f(P3d.x,P3d.y), buildline_centerpt);
            //用点到线段的距离-------------------------------------------
            dis = Config::PointToSegDist(P3d.x, P3d.y, eachseenfacade[0].x, eachseenfacade[0].y, eachseenfacade[1].x, eachseenfacade[1].y);
            if(dis < min_distance)
            {
                min_distance = dis;
                cloest_facade = eachseenfacade;
                record_closet_buildnum = seen;
            }
        }
        return cloest_facade;
    }
    
    vector<cv::Point3f> Config2::find_closest_facade2(const cv::Point3f& P3d, map<int,map<int,vector<cv::Point3f>>>& map_buildings, Config2::PointSet all_seenindex, float& dis_out)
    {
        map<int, vector<cv::Point3f>> cloest_facade_;
        vector<cv::Point3f> cloest_facade;
        double min_distance = 100000;
        cv::Point2i record_closet_buildnum;
        for (ORB_SLAM2::Config2::PointSet::iterator it=all_seenindex.begin(); it!=all_seenindex.end(); ++it)
        {
            cv::Point2i seen;
            seen = *it;
            vector<cv::Point3f> eachseenfacade = map_buildings[seen.x][seen.y];
            cv::Point3f normalvector_of_facade, pt_on_facade;
            pt_on_facade = eachseenfacade[4];
            normalvector_of_facade = eachseenfacade[5];
            
            double dis;
            cv::Point2f buildline_centerpt;
            buildline_centerpt.x = (eachseenfacade[0].x+eachseenfacade[1].x)/2;
            buildline_centerpt.y = (eachseenfacade[0].y+eachseenfacade[1].y)/2;
            //用点到线段的距离-------------------------------------------
            dis = Config::PointToSegDist(P3d.x, P3d.y, eachseenfacade[0].x, eachseenfacade[0].y, eachseenfacade[1].x, eachseenfacade[1].y);
            if(dis < min_distance)
            {
                min_distance = dis;
                cloest_facade = eachseenfacade;
                record_closet_buildnum = seen;
            }
        }
        if(min_distance < 100000)
        {
            dis_out = min_distance;
        }
        else
            dis_out = 0;
        return cloest_facade;
    }

}

