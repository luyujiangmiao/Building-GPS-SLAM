#include"Read2_5Dmap.h"
#include<iostream>


#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/features2d/features2d.hpp>

void Read2_5map::LoadObjData(const string &sobj, vector<cv::Point3d>& vertics,
                             vector<cv::Point3d>& normals,  vector<vector<cv::Point3i>>& faceData,
                             /*vector<cv::Point2d>& vertics_ground, vector<vector<cv::Point3i>>& faceData_ground,*/ int& facades_num)
{
    //string objectName = null;
    int faceDataCount = 0;

    string::const_iterator it=sobj.begin();
    string s;
    char c;
    cv::Point2d center_pt;
    for(;it != sobj.end();it++)
    {

        //meet \n save a line
        c = *it;
        if(c != '\n'){
            s += c;
        }
        else{

        if(s[0]=='#')
        {
            if(s[2]=='C'&& s[3]=='e')
            {
                int splitStart = 9;
                string str1, str2;
//                double center1, center2;
                //cut head cut tail
                s = s.substr(splitStart,s.length()-1);

                istringstream is(s);
                is>>str1>>str2;
                stringstream str1_, str2_;
                str1_ << str1;
                str2_ << str2;

                str1_ >> center_pt.x;
                str2_ >> center_pt.y;
            }

        }
        else if(s[0]=='v' && s[1]==' ')// Vertices
        {
            int splitStart = 2;
            string str1,str2,str3;

            //cut head cut tail
            s = s.substr(splitStart,s.length()-1);

            //meet ' ' save a number
            istringstream is(s);
            is>>str1>>str2>>str3;
            stringstream str1_,str2_,str3_;
            str1_ << str1;
            str2_ << str2;
            str3_ << str3;

            cv::Point3d v;
            str1_ >> v.x;
            str2_ >> v.y;
            str3_ >> v.z;

            v.x=v.x+center_pt.x-534796.31;
            v.y=v.y+center_pt.y-5211807.79;// world coordinate system

            if(v.z>0)
            {
                v.z=v.z-357;
            }


            vertics.push_back(v);

        }
        else if(s[0] == 'v' && s[1]== 'n' && s[2]==' ') //normal
        {
            int splitStart = 3;
            string str1,str2,str3;

            //cut head cut tail
            s = s.substr(splitStart,s.length()-1);

            istringstream is(s);
            is>>str1>>str2>>str3;
            stringstream str1_,str2_,str3_;
            str1_ << str1;
            str2_ << str2;
            str3_ << str3;

            cv::Point3d n;
            str1_ >> n.x;
            str2_ >> n.y;
            str3_ >> n.z;

            normals.push_back(n);

        }
        else if(s[0] == 'f' && s[1]==' ')//face
        {
            int splitStart = 2;
            string str1,str2,str3;

            s = s.substr(splitStart,s.length()-1);

            istringstream is(s);
            is>>str1>>str2>>str3;

            stringstream str1_,str2_,str3_;
            str1_ << str1;
            str2_ << str2;
            str3_ << str3;
            int f_1,f_2,f_3;
            str1_ >> f_1;
            str2_ >> f_2;
            str3_ >> f_3;


            cv::Point3i each1,each2,each3;

            for(int i=0; i!=str1.length();i++)
            {
                if(str1[i] == '/' && str1[i+1] == '/')
                {
                    stringstream vert_s, norm_s;

                    vert_s << str1.substr(0, i);
                    vert_s >> each1.x;
                    each1.y = 0;
                    norm_s << str1.substr(i+2, str1.length() - i - 2);
                    norm_s >>each1.z;
                    break;
                }

            }

            for(int i=0; i!=str2.length();i++)
            {
                if(str2[i] == '/' && str2[i+1] == '/')
                {
                    stringstream vert_s, norm_s;

                    vert_s << str2.substr(0, i);
                    vert_s >> each2.x;
                    each2.y = 0;
                    norm_s << str2.substr(i+2, str2.length() - i - 2);
                    norm_s >> each2.z;
                    break;
                }

            }

            for(int i=0; i!=str3.length();i++)
            {
                if(str3[i] == '/' && str3[i+1] == '/')
                {
                    stringstream vert_s, norm_s;
                    vert_s << str3.substr(0, i);
                    vert_s >> each3.x;
                    each3.y = 0;
                    norm_s << str3.substr(i+2, str3.length() - i - 2);
                    norm_s >> each3.z;

                    //cal the facade number
                    if(facades_num < each3.z){
                        facades_num = each3.z;
                        }
                    break;
                }

            }


            vector<cv:: Point3i> f;
            f.push_back(each1);
            f.push_back(each2);
            f.push_back(each3);
            faceData.push_back(f);
            faceDataCount++;
        }
        s = "";

        }

    }


}

void Read2_5map::Parse_json(const string& strjsonfilePath)
{
    std::ifstream file(strjsonfilePath.c_str());
    jsonxx::Object o;
    o.parse(file);
    cout<<o.get<jsonxx::String>("status")<<"\n";
    cout<<o.get<jsonxx::String>("message")<<"\n";
    jsonxx::Array data = o.get<jsonxx::Array>("data");
    cout<<"building numbers: "<<data.size()<<"\n";
    json_size = (int)data.size();

    for(int i=0; i<data.size();i++)
    {
        int gid = data.get<jsonxx::Object>(i).get<jsonxx::Number>("gid");
        double x = data.get<jsonxx::Object>(i).get<jsonxx::Number>("x");
        double y = data.get<jsonxx::Object>(i).get<jsonxx::Number>("y");
        string obj = data.get<jsonxx::Object>(i).get<jsonxx::String>("obj");

        vector<cv::Point3d> vertics;
        vector<cv::Point3d> normals;
        vector<vector<cv::Point3i>> faceData;


        int facades_num;

        LoadObjData(obj, vertics, normals, faceData, facades_num);


        map_vertics[i] = vertics;
        map_normals[i] = normals;
        map_faceData[i] = faceData;

        facadesnum_eachbuilding[i] = facades_num;

    }
}



Read2_5map::Read2_5map(const string & path){
    scale = 0.001;
    Parse_json(path);
    std::cout<<"load finish\n";
}


void Read2_5map::save_obj(string path,const cv::Mat& Tc1w_formap) {
    ofstream out(path);
    for (auto i = 0; i < json_size; i++) {
        for (auto vi = 0; vi < map_vertics[i].size(); vi++) {
            cv::Point3f temp;
            temp.x = float(map_vertics[i][vi].x);
            temp.y = float(map_vertics[i][vi].y);
            temp.z = float(map_vertics[i][vi].z);
            temp = ORB_SLAM2::Config2::transPw2Pc1(temp,Tc1w_formap);

            temp.x = temp.x*scale;
            temp.y = temp.y*scale;
            temp.z = temp.z*scale;

            out << "v " << temp.x << " " << temp.y << " "
            << temp.z << "\n";
        }
        
    }
    cout << "\n";
    
    for (auto i = 0; i < json_size; i++) {
        for (auto vi = 0; vi < map_normals[i].size(); vi++) {
            out << "v " << map_normals[i][vi].x << " " << map_normals[i][vi].y << " "
            << map_normals[i][vi].z << "\n";
        }
    }
    
    cout << "\n";
    int cnt_f = 0;
    int cnt_v = 0;
    for (auto i = 0; i < json_size; i++) {
        for (auto fi = 0; fi < map_faceData[i].size(); fi++) {
            auto& face = map_faceData[i][fi];
            
            out << "f " << face[0].x + cnt_v << "//" << face[0].z + cnt_f << " "
            << face[1].x + cnt_v << "//" << face[1].z + cnt_f << " "
            << face[2].x + cnt_v << "//" << face[2].z + cnt_f << "\n";
        }
        cnt_v += map_vertics[i].size();
        cnt_f += map_faceData[i].size();
    }
    out.close();
}

