#include"Config.h"
namespace ORB_SLAM2 {
    cv::Point2i Config::Fn_Projection_pt(cv::Point3f points3f, cv::Mat ca_Tcc1)
    {
        
        vector<cv::Point2i> AR_ps;

        //Step1
//        cout<<"the 3D point of AR model: "<<points3f<<endl;
//        cv::Point3f Unified_AR_P;
//        Unified_AR_P = Config::trans_point3d_ruyu(points3f, initialpose)*0.001;
        
        //Step2
        cv::Mat ca_Rcc1,ca_tcc1;
        ca_Tcc1.convertTo(ca_Tcc1, CV_32F);

        ca_Rcc1 = ca_Tcc1.rowRange(0, 3).colRange(0, 3);
        ca_tcc1 = ca_Tcc1.rowRange(0, 3).col(3);
        
        cv::Mat v1_(3,1,CV_32F),v2_(3,1,CV_32F);
        v1_ = v1_.t();
        v2_ = v2_.t();
        v1_.at<float>(0) = points3f.x;
        v1_.at<float>(1) = points3f.y;
        v1_.at<float>(2) = points3f.z;
        
        ca_tcc1 = ca_tcc1.t();
        cv::Point3f AR_Pc;
        //AR_Pc = Rcw*Unified_AR_P+tcw;
        v2_.at<float>(0) = ca_Rcc1.row(0).dot(v1_)+ca_tcc1.at<float>(0);
        v2_.at<float>(1) = ca_Rcc1.row(1).dot(v1_)+ca_tcc1.at<float>(1);
        v2_.at<float>(2) = ca_Rcc1.row(2).dot(v1_)+ca_tcc1.at<float>(2);
        AR_Pc.x = v2_.at<float>(0);
        AR_Pc.y = v2_.at<float>(1);
        AR_Pc.z = v2_.at<float>(2);
        //Step3
        //            Camera.fx: 618.126667
        //            Camera.fy: 619.2333
        //            Camera.cx: 330.64
        //            Camera.cy: 175.5466667
        float fx = 618.126667;
        float fy = 619.2333;
        float cx = 330.64;
        float cy = 175.5466667;
        float invz = 1.0f/AR_Pc.z;
        float u=fx*AR_Pc.x*invz+cx;
        float v=fy*AR_Pc.y*invz+cy;
        cv::Point2i AR_p;
        AR_p.x = (int) u;
        AR_p.y = (int) v;
    
        return AR_p;
    }

    cv::Mat Config::Fn_ProjectionAR(cv::Mat img, int id, cv::Mat ARmodel, vector<cv::Point3f> facade, cv::Mat ca_Tcw)
    {
        
//        cv::imwrite("/Users/vision/Downloads/luyu/project/ORB_SLAM_xcode_graz/Examples/results/logo_copy.jpg", ARmodel);
//        cout<<"channel of AR img: "<<ARmodel.channels()<<endl;
        cv::Mat facadewithlog;
        img.copyTo(facadewithlog);
        float fx = 618.126667;
        float fy = 619.2333;
        float cx = 330.64;
        float cy = 175.5466667;

        cv::Mat ca_Rcw,ca_tcw;
        ca_Tcw.convertTo(ca_Tcw, CV_32F);
        
        ca_Rcw = ca_Tcw.rowRange(0, 3).colRange(0, 3);
        ca_tcw = ca_Tcw.rowRange(0, 3).col(3);
        
        cv::Mat v1_(3,1,CV_32F),v2_(3,1,CV_32F);
        v1_ = v1_.t();
        v2_ = v2_.t();
        cv::Point3f AR_Pc[4];
        for(int index =0; index <5;index++)
        {
            if(index == 2)
                continue;
          
            v1_.at<float>(0) = facade[index].x;
            v1_.at<float>(1) = facade[index].y;
            v1_.at<float>(2) = facade[index].z;
            
            ca_tcw = ca_tcw.t();
//            cout<<"ca_Rcw: "<<ca_Rcw<<endl;
//            cout<<"ca_tcw: "<<ca_tcw<<endl;
//            cout<<"v1_: "<<v1_<<endl;
            //AR_Pc = Rcw*Unified_AR_P+tcw;
            v2_.at<float>(0) = ca_Rcw.row(0).dot(v1_)+ca_tcw.at<float>(0);
            v2_.at<float>(1) = ca_Rcw.row(1).dot(v1_)+ca_tcw.at<float>(1);
            v2_.at<float>(2) = ca_Rcw.row(2).dot(v1_)+ca_tcw.at<float>(2);
            AR_Pc[index].x = v2_.at<float>(0);
            AR_Pc[index].y = v2_.at<float>(1);
            AR_Pc[index].z = v2_.at<float>(2);
//            cout<<"AR_Pc: "<<AR_Pc[index]<<endl;
            
        }
       
        //左上-右下【2】-【1】，【3】-【0】斜对角
        cv::Point3f zuoshang, youxia;
        zuoshang = AR_Pc[3];
        youxia = AR_Pc[0];

        //or
//        zuoshang = AR_Pc[4];
//        youxia = AR_Pcl[3];
//        double k;
//        k = (youxia.z-zuoshang.z)/(youxia.x-zuoshang.x); //z is depth, y is height
        
        double alpha,beta;
        Vec3b pixel_AR;
        cv::Point3f AR_Pc_;
        float invz;
        cv::Point2i AR_p;
        for(int i =0;i<ARmodel.rows;i++)//所在行y坐标 225
        {
            for(int j=0;j<ARmodel.cols;j++)//所在行x坐标 225
            {
                pixel_AR = ARmodel.at<Vec3b>(i,j);
                alpha = i*1.0/ARmodel.rows;//height
                beta = j*1.0/ARmodel.cols;

                AR_Pc_.x = zuoshang.x + beta*(youxia.x-zuoshang.x);
                AR_Pc_.z = zuoshang.z + beta*(youxia.z-zuoshang.z);
                AR_Pc_.y = zuoshang.y + alpha*(youxia.y-zuoshang.y);

                invz = 1.0f/AR_Pc_.z;
                float u=fx*AR_Pc_.x*invz+cx;
                float v=fy*AR_Pc_.y*invz+cy;
                AR_p.x = (int) u;
                AR_p.y = (int) v;
                if( AR_p.y<360 && AR_p.x<640 && AR_p.y>0 && AR_p.x>0)
                facadewithlog.at<Vec3b>(AR_p.y,AR_p.x) = pixel_AR;//row,col
            }
        }

//        
//        cv::imwrite("/Users/vision/Downloads/luyu/project/ORB_SLAM_xcode_graz/Examples/results2-07-08/facadewithlogo.jpg", facadewithlog);
        return facadewithlog;
    }

    
    cv::Point3f Config::vec2pt3d(const vector<double> buildingline)
    {
        cv::Point3f buildingline_;
        buildingline_.x = buildingline[0];
        buildingline_.y = buildingline[1];
        buildingline_.z = buildingline[2];
        return buildingline_;
    }

    

//点到线段的距离公式
    double Config::PointToSegDist(double x, double y, double x1, double y1, double x2, double y2)
    {
        double cross = (x2-x1)*(x-x1)+(y2-y1)*(y-y1);
        if(cross<=0) return sqrt((x-x1)*(x-x1)+(y-y1)*(y-y1));
        
        double d2 = (x2-x1)*(x2-x1)+(y2-y1)*(y2-y1);
        if(cross >= d2) return sqrt(x-x2)*(x-x2)+(y-y2)*(y-y2);
        
        double r = cross/d2;
        double px = x1+(x2-x1)*r;
        double py = y1+(y2-y1)*r;
        return sqrt((x-px)*(x-px)+(py-y)*(py-y));
    }
    
//两点之间距离公式
    double Config::cal_distance2(cv::Point2f pt1, cv::Point2f pt2)
    {
        double distance;
        distance = sqrt((pt1.x-pt2.x)*(pt1.x-pt2.x)+(pt1.y-pt2.y)*(pt1.y-pt2.y));
        return distance;
    }
    
    void Config::mergeImg(cv::Mat &dst, cv::Mat src1, cv::Mat src2)
    {
        int rows = src1.rows+src2.rows;
        int cols = src1.cols;
        dst.create(rows,cols,src1.type());
        src1.copyTo(dst(Rect(0,0,src1.cols,src1.rows)));
        src2.copyTo(dst(Rect(0,src1.rows,src2.cols,src2.rows)));
    }
    
    //求建筑线的直线方程
    vector<double> Config::cal_buildingline(const cv::Point2d &pt1, const cv::Point2d &pt2)
    {
        vector<double> buildingline;
        double temp_k,temp_b,temp_x0;
        if(abs(pt1.x-pt2.x)<eps)
        {
            temp_k = 100000;
            temp_b = 100000;
            temp_x0 = pt1.x;
        }
        else
        {
            temp_k = (pt2.y-pt1.y)/(pt2.x-pt1.x);
            temp_b = pt1.y-temp_k*pt1.x;
            temp_x0 = 100000;
        }
        buildingline.push_back(temp_k);
        buildingline.push_back(temp_b);
        buildingline.push_back(temp_x0);
        return buildingline;
    }
    
    
    //added by ruyu
    void Config::cal_cross(const cv::Point3f u, const cv::Point3f v, cv::Point3f& cross)
    {
        //a2b3-a3b2,a3b1-a1b3, a1b2-a2b1]
        cross.x = u.y*v.z-u.z*v.y;
        cross.y = u.z*v.x-u.x*v.z;
        cross.z = u.x*v.y-u.y*v.x;
        
        //normal
        double norm = sqrt(cross.x*cross.x+ cross.y*cross.y+ cross.z*cross.z);
        cross.x = cross.x/norm;
        cross.y = cross.y/norm;
        cross.z = cross.z/norm;
    }
    

    //cal the building facade using one point anf vector
    map<int,map<int,vector<cv::Point3f>>> Config::cal_2dbuilding(const string &mapfilepath, const cv::Mat& Tc1w_formap)
    {
        map<int,map<int,vector<cv::Point3f>>>map_buildings;
        cv::Mat showimg(600, 600, CV_8UC3, Scalar(255,255,255));//bgr

        Read2_5map map2_5d(mapfilepath);
//        string savepath = "/Users/vision/Downloads/luyu/project/ORB_SLAM_xcode_graz/Examples/results2-02-21/TUgraz.obj";
//        map2_5d.save_obj(savepath,Tc1w_formap);
        
        map<int,vector<vector<cv::Point3i>>>::iterator it;
        it = map2_5d.map_faceData.begin();
        int buildingcount = 0;
        while(it != map2_5d.map_faceData.end())
        {
            

            map<int,vector<cv::Point3f>> eachbuilding;

            cv::Point3f record_firstpt_ofbuid;
            int j_count=0;
            for(int j=0;j<it->second.size()-1;j=j+2)
            {
                vector<cv::Point3f> facade_equation;
                vector<cv::Point3i> f,ff;

                f = it->second[j];
                ff = it->second[j+1];

                int pt1_idex, pt2_idex, pt3_idex, pt4_idex;
                pt1_idex = f[0].x-1;//-1因为vector从0开始计数，而json文件从1开始
                pt2_idex = f[1].x-1;
                pt3_idex = f[2].x-1;
                
                for(int index=0;index<3;index++)
                {
                    if((ff[index].x != pt1_idex) && (ff[index].x != pt2_idex) && (ff[index].x != pt3_idex))
                    {
                        pt4_idex = ff[index].x;
                        break;
                    }
                    
                }
                
                //find building pts on ground
                cv::Point3f pt1,pt2, pt3,pt4;
                cv::Point3f pt1_ground, pt2_ground, pt3_up, pt4_up;
                pt1 = map2_5d.map_vertics[buildingcount][pt1_idex];
                pt2 = map2_5d.map_vertics[buildingcount][pt2_idex];
                pt3 = map2_5d.map_vertics[buildingcount][pt3_idex];
                pt4 = map2_5d.map_vertics[buildingcount][pt4_idex];
                if(pt1.z ==0)
                {
                    pt1_ground = pt1;
                    if(pt2.z==0)
                    {
                       pt2_ground = pt2;
                       pt3_up = pt3;
                       pt4_up = pt4;
                    }
                    else if(pt3.z==0)
                    {
                       pt2_ground = pt3;
                        pt3_up = pt2;
                        pt4_up = pt4;
                    }
                    else
                    {
                        pt2_ground = pt4;
                        pt3_up = pt2;
                        pt4_up = pt3;
                    }
                }
                else if(pt2.z ==0)
                {
                    pt1_ground = pt2;
                    if(pt3.z==0)
                    {
                        pt2_ground = pt3;
                        pt3_up = pt1;
                        pt4_up = pt4;
                    }
                    else
                    {
                        pt2_ground = pt4;
                        pt3_up = pt1;
                        pt4_up = pt3;
                    }
              
                }
                else if(pt3.z  ==0)
                {
                    pt1_ground = pt3;
                    pt2_ground  = pt4;
                    pt3_up = pt1;
                    pt4_up = pt2;
                }
                

                pt1_ground = Config2::transPw2Pc1(pt1_ground,Tc1w_formap)*0.001;//x,z,y
                pt2_ground = Config2::transPw2Pc1(pt2_ground,Tc1w_formap)*0.001;
                pt3_up = Config2::transPw2Pc1(pt3_up,Tc1w_formap)*0.001;
                pt4_up = Config2::transPw2Pc1(pt4_up,Tc1w_formap)*0.001;

                cv::Point3f u,v,cross;
                u.x = pt2_ground.x-pt1_ground.x;
                u.y = pt2_ground.y-pt1_ground.y;
                u.z = pt2_ground.z-pt1_ground.z;
                
                v.x = pt3_up.x-pt1_ground.x;
                v.y = pt3_up.y-pt1_ground.y;
                v.z = pt3_up.z-pt1_ground.z;
                
                cal_cross(u,v,cross);

                
                if(j == 0)
                {
                    record_firstpt_ofbuid = pt2_ground;
                }
 
                vector<double> facadeline_;
                cv::Point3d facadeline;

                facadeline_ = cal_buildingline(cv::Point2d(pt1_ground.x,pt1_ground.z),cv::Point2d(pt2_ground.x,pt2_ground.z));

                facadeline = vec2pt3d(facadeline_);
                facade_equation.push_back(pt1_ground);//0
                facade_equation.push_back(pt2_ground);//1
                facade_equation.push_back(facadeline);//2地面上的直线方程
                facade_equation.push_back(pt3_up);//3
                facade_equation.push_back(pt4_up);//4
                facade_equation.push_back(cross);//5法向量
                eachbuilding[j_count] = facade_equation;
                
                j_count++;
            }
            map_buildings[buildingcount] = eachbuilding;
            buildingcount++;
            it ++;
        }
        return map_buildings;
    }
 
    
    //求交点 y-x坐标系下的
    cv::Point2d Config::cal_insetpt(const vector<double>& eyeline, const vector<double>&buildingline)
    {
        cv::Point2d insectpt;
        //[0]:k,[1]:b
        //两条竖线或者平行
        if((eyeline[0]==100000 && buildingline[0]==100000)|| abs(eyeline[0] - buildingline[0]) < 0.0001)
        {
            cout<<"no insection."<<"\n";
        }
        
        else{
            if(buildingline[0] == 100000)//建筑物线是竖线
            {
                insectpt.x = buildingline[2];
                insectpt.y = eyeline[0]*insectpt.x+eyeline[1];
            }
            else//两条都不是竖线
            {
                insectpt.x = (buildingline[1]-eyeline[1])/(eyeline[0]-buildingline[0]);
                insectpt.y = eyeline[0]*insectpt.x+eyeline[1];
            }
        }
        return insectpt;
    }
    
    
    
    double Config::eps = 1e-8;
    int Config::sgn(double x){
        if(x<-eps) return -1;
        if(x>eps) return 1;
        else return 0;
    }
    int Config::Judge_insectpt1(cv::Point2d insectpt,  cv::Point2d pt1, cv::Point2d pt2){

        if(sgn(pt1.x - pt2.x) ==0){
            if(sgn(pt1.y - insectpt.y) * sgn(pt2.y - insectpt.y) > 0) return 0;
        }else if(sgn(pt1.x - insectpt.x) * sgn(pt2.x-insectpt.x) > 0) return 0;
        return 1;//insection point on line
    }
    

}


