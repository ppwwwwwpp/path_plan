#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <cstring>
#include "bp_std_msgs/AlongWallBow.h"
#include "bp_std_msgs/AlongWallBowTask.h"
#include "bp_std_msgs/PathArea.h"
#include "bp_std_msgs/VirtualWall.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <libgen.h>
#include <sys/types.h>
#include <dirent.h>
#include "yaml-cpp/yaml.h"
#include <SDL/SDL_image.h>
#include <fstream>
#include "std_msgs/UInt32.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include <sstream>

#define K_a 12 //沿墙像素膨胀
#define K_b 24 //弓形像素膨胀  
#define interval 7 //弓形布线间隔 
#define N_point 100 //沿墙最小路径过滤
#define preset_num 500 //
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))
#define End 3

#ifdef HAVE_NEW_YAMLCPP
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template <typename T>
void operator>>(const YAML::Node &node, T &i)
{
  i = node.as<T>();
}
#endif

using namespace std;
using namespace cv;

//定义结构体
typedef struct {
	vector<vector<int> >way_x;
	vector<vector<int> >way_y;
}WPList;

typedef struct {
	vector<int>x;
	vector<int>y;
}PathList;

typedef struct{
	int x;
	int y;
}newAfterRotation;

typedef struct {
	int** edgeway;
	int num;
}Edgeway;

typedef struct {
	int*** Path;
	int Num;
	vector<int>NumOfPoint;
}WallPath;

typedef struct {
	string name;
	std::vector<geometry_msgs::Pose> bow;
	std::vector<geometry_msgs::Pose> wall;
	std::vector<geometry_msgs::PoseArray> wall_sec;
}MapResult;

typedef struct{
	PathList wall_list;
	WallPath wall_path;
	unsigned char** p;
}WallSolution;

//函数声明
//路径覆盖
WPList getWaypointList_w(int step10, int** pr, int h, int w);
PathList maxLengthPath_w(int MinLeng, int** pr, int h, int w);
WPList getWaypointList_h(int step10, int** pr, int h, int w);
PathList maxLengthPath_h(int MinLeng, int** pr, int h, int w);
PathList bowFinding(Mat image_origin, double angle, int Y_, int X_);
//dijkstra
int** dijkstra(int** p, int y, int x,int h,int w);
void dijkstra02(int** p, int y, int x, int h, int w);
PathList backtrace(int x, int y, int**p, Mat image );
//沿墙
Edgeway edgeway_path(int** img, int h, int w, int x, int y);
WallPath dijAlong(int** pr, int h, int w, int x, int y);
PathList pathSorted(PathList getWaypointList, int** p, Mat erode_img_save);
PathList pointFinding(Mat erode_img, int**p, int** pr);
void showResult(PathList s_list, Mat img, Mat image, double angle);
int length(PathList len);
newAfterRotation rotation(int y,int x,double angle,int h,int w);
WallSolution wallFinding(Mat image_origin,int Y, int X);

//读取地图
void testMapDir(const char *filepath, string name_map,Mat& image_origin,double& ori_x, double& ori_y);
bool getMapFileInfo(std::string fname, std::string &image, double &resolution, double &origin_x, double &origin_y, double &origin_z, int &negate, double &occupied_thresh, double &free_thresh);

int wall_check=0;

//类定义
class PathFindingClass
{
	public:
		PathFindingClass()
		{
			pub_=n_.advertise<bp_std_msgs::AlongWallBow>("bp_nav_to_android_alongwallbow",1);
			// pub_test=n_.advertise<std_msgs::UInt32>("bp_nav_to_android_alongwallbow",1);
			pub_debug_bow=n_.advertise<geometry_msgs::PoseArray>("path_finding_bow_debug",1,true);
			pub_debug_wall=n_.advertise<geometry_msgs::PoseArray>("path_finding_along_wall_debug",1,true);
                        
			pub_active=n_.advertise<std_msgs::UInt32>("bp_node_feedback",1);
			pub_rec=n_.advertise<bp_std_msgs::PathArea>("bp_path_area_draw",1,true);
			pub_vir=n_.advertise<bp_std_msgs::VirtualWall>("bp_virtual_wall_read",1,false);

                        sub_=n_.subscribe("/bp_android_to_nav_alongwallbow",1,&PathFindingClass::pathCallback,this);
			sub_pos=n_.subscribe("/robot_pose",1,&PathFindingClass::startPoint,this);

			sub_rec=n_.subscribe("/bp_area_draw",1,&PathFindingClass::drawCallback,this);


		}

		//ROS回调函数
                void pathCallback(const bp_std_msgs::AlongWallBowTask::ConstPtr& msg);
		void startPoint(const geometry_msgs::Pose::ConstPtr& msg);
		void drawCallback(const bp_std_msgs::PathArea::ConstPtr& msg);
		bool calculatePath(Mat image_origin, double ori_x, double ori_y, MapResult& path_);
		//void deleteArea(const geometry_msgs::Polygon::ConstPtr& msg);

	private:
		ros::NodeHandle n_;
		ros::Publisher pub_;
		ros::Publisher pub_test;
		ros::Publisher pub_debug_wall;
		ros::Publisher pub_debug_bow;
		ros::Publisher pub_active;
		ros::Publisher pub_rec;
		ros::Subscriber sub_;
		ros::Subscriber sub_pos;
		ros::Subscriber sub_ask;
		ros::Subscriber sub_rec;
		ros::Subscriber sub_vir;
		ros::Subscriber sub_vir_msg;	
		ros::Publisher pub_vir;
		//ros::Subscriber sub_delete;
		double Y ;
		double X ;
		MapResult path_;
		bp_std_msgs::AlongWallBow P,P_save_a;
		bp_std_msgs::PathArea path_rec;
		string save;
		Mat image_edited;
		Mat image_origin;
		double ori_y;
		double ori_x;	
		bool flag_edit;	
		bool flag_read;
		bool flag;
		
};

int main(int argc,char** argv)
{
    ros::init(argc,argv,"path_finding");
	PathFindingClass MyPath;
    ros::spin();
    return 0;
}



bool PathFindingClass::calculatePath(Mat image, double ori_x, double ori_y, MapResult& path_)
{
	
	if (image.data==0)  
	{
		ROS_ERROR("Map Not Ready");
		return false;
	}

	 //string path1="/home/lx/debug/tem";
	 //string pathofedit = path1+".txt";
	 //ofstream test;
	 //test.open(pathofedit.c_str());
	 //test<<"entered"<<endl;
	 //test.close();

	path_.bow.clear();
	path_.wall.clear();
	geometry_msgs::Pose p1,p2;
	geometry_msgs::Pose temp;
	geometry_msgs::PoseArray P_array;

	const int h = image.rows;  
	const int w = image.cols;

	cout<<"Hight: h="<<h<<" Width: w="<<w<<endl;

	int Y_,X_; //dijkstra进入点
        
        X = 0.0;
        Y = 0.0;

        X_=h-(Y-ori_x)/0.05;
        Y_=(X-ori_y)/0.05;   

	 //Y_=150;

	 //X_=150;
	

	cout<<"position on map: X_="<<X_<<" "<<"Y_="<<Y_<<endl;
	if(X_<0||Y_<0)
		return false;
	////////////////////计算弓型线/////////////////////
	int theOne=0;
        int pathlength;
	int it;
	PathList s_list_p;
	Mat rot_mat, erode_img, erode_element, re_img, img;
	int angleT;
	angleT=atan2(h,w)*180/M_PI;

	int c=0;
	vector<double>angle;
		
	for (int i=-angleT;i<angleT;i++)
	{
		angle.push_back(i);
	}


	for(int k=0;k<angle.size();k++)
	{	

	        int i,j;
	
                cout<<"------"<<"AngleOfRotation:"<<angle[k]<<"------"<<endl;


		PathList s_list = bowFinding(image, angle[k], Y_, X_);
		
		pathlength=length(s_list);

		ROS_INFO("TheNumOfBowList: %d",s_list.x.size());

		if(s_list.x.size()==1)
		{
			ROS_ERROR("PosesInGrey!!!");
			continue;
		}
			

		if(s_list.x.size()==2)
		{
			ROS_INFO("SKIP!");
			continue;
		}	

		if(s_list.x.size()==3)
		{
			return false;
		}



		if(pathlength>theOne)
		{
			theOne=pathlength;
			it=k;
			s_list_p=s_list;
                      //cout<<"pathlength"<<endl;
		}

	}

	if(theOne==0)
	{
		ROS_ERROR("AllTryPosesInGrey!!!");
		return false;
	}
		

	cout<<"AngleOfMaximumCoverage:"<<angle[it]<<endl;

			
		for (int i = 0; i < s_list_p.x.size(); i++) {
			newAfterRotation point=rotation(s_list_p.y[i],s_list_p.x[i],angle[it],h,w);
			Point p(point.x,point.y);

			// circle(image_origin,p,5,Scalar(200,60,60),-1);

			p1.position.y=(h-point.y-1)*0.05+ori_x;
			p1.position.x=point.x*0.05+ori_y;
			p1.orientation.w=1.0;
			path_.bow.push_back(p1);

                        //cout<<path_.bow.size()<<endl;       
		}
		//imshow("image", image_origin);
		//waitKey();
	        //destroyAllWindows();


                ROS_INFO("path_bow_size: %lu",path_.bow.size());

                ROS_INFO("Bow Done Wall Started");  
  
	/*	if(!path_.bow.empty()) 
		{
			ROS_INFO("Bow Successful!");
			return true;
		}
			
		else 
		{
			ROS_ERROR("BowPath is EMPTY");
			return false;
		}  
      */
		////////////////////////计算沿墙路径/////////////////////////////////////	
                ROS_INFO("Wall Started");  
		Mat erode_img2,erode_element2;
		erode_element2=getStructuringElement(MORPH_ELLIPSE, Size(K_a, K_a));
		// erode_element2=getStructuringElement(MORPH_RECT, Size(K_a, K_a));
		erode(image,erode_img2,erode_element2);
		

		WallSolution wall_temp_0 = wallFinding(erode_img2,Y_, X_);
		Mat image_wall;
		image_wall=erode_img2.clone();


		for(int i=0; i<wall_temp_0.wall_list.x.size();++i)
		{
			Point p(wall_temp_0.wall_list.y[i], wall_temp_0.wall_list.x[i]);
			circle(image_wall, p, End, Scalar(0, 0, 0)); 
		}
	        //imshow("image",image_wall);
		//waitKey();
		WallSolution wall_temp = wallFinding(image_wall,Y_, X_);

		if(wall_check==1)
			return false;


		vector<int>tempx;
		vector<int>tempy;
		vector<int>s_listx;
		vector<int>s_listy;
		vector<int>checkx;
		vector<int>checky;
		vector<int>path_tx;
		vector<int>path_ty;

		tempx=wall_temp.wall_list.x;
		tempy=wall_temp.wall_list.y;
		checkx=tempx;
		checky=tempy;

		WallPath pathFinding = wall_temp.wall_path;
		unsigned char** pp = wall_temp.p;

		//////////////路径
		vector<vector<double> >mypathx;
		vector<vector<double> >mypathy;
		for(int i=0;i<pathFinding.Num+pathFinding.Num-1;++i)
		{
			vector<double>temp_x; 
			vector<double>temp_y; 
			mypathx.push_back(temp_x);
			mypathy.push_back(temp_y);
		}
		///////////////////

		int index=0; //路径分段索引号
		
		for (int i=0;i<pathFinding.NumOfPoint[0]-1;i++)
		{
			path_tx.push_back(pathFinding.Path[0][0][i]);
			path_ty.push_back(pathFinding.Path[0][1][i]);

			// circle(image_origin,(Point(pathFinding.Path[0][1][i],pathFinding.Path[0][0][i])),3,Scalar(120.120,120),-1);
			// imshow("image", image_origin);
			// waitKey(2);

			mypathx[index].push_back((h-pathFinding.Path[0][0][i])*0.05+ori_x);
			mypathy[index].push_back(pathFinding.Path[0][1][i]*0.05+ori_y);
		}

		// imshow("image", image_origin);
		// waitKey();

		index=index+1;

		for(int i=0;i<tempx.size();i++)
			pp[tempx[i]][tempy[i]]=150;

		pp[tempx[0]][tempy[0]] = 254;

		s_listx.push_back(tempx[0]);
		s_listy.push_back(tempy[0]);

		tempx.erase(tempx.begin());
		tempy.erase(tempy.begin());

		pp[tempx[0]][tempy[0]] = 254;

		s_listx.push_back(tempx[0]);
		s_listy.push_back(tempy[0]);

		tempx.erase(tempx.begin());
		tempy.erase(tempy.begin());

		vector<int>cx;
		vector<int>cy;
		vector<int>templistx;
		vector<int>templisty;

		int x, y;
		int** p_save = new int* [h];

		for (int i = 0; i < h; i++)
			p_save[i] = new int[w];

		int path_length;
		

		while (!tempx.empty())
		{
			x = s_listx.back();
			y = s_listy.back();

			path_length = 1000;

			for (int i = 0; i < h; i++)
			{
				for (int j = 0; j < w; j++)
				{
					p_save[i][j] = erode_img2.at<Vec3b>(i, j)[0];

				}
			}

			p_save[x][y] = path_length;
			templistx.clear();
			templisty.clear(); 
			templistx.push_back(x);
			templisty.push_back(y);
			vector<int>endx;
			vector<int>endy;

			while (!templistx.empty())
			{
				cx.clear();
				cy.clear();
				path_length += 1;
				for (int i = 0; i < templistx.size(); i++) {
					x = templistx[i];
					y = templisty[i];
					if (pp[x][y] == 150){
						endx.push_back(x);
						endy.push_back(y);
						break;
					}
					if (p_save[x][y - 1] == 254)
					{
						p_save[x][y - 1] = path_length;
						cx.push_back(x);
						cy.push_back(y - 1);
					}
					if (p_save[x - 1][y] == 254)
					{
						p_save[x - 1][y] = path_length;
						cx.push_back(x - 1);
						cy.push_back(y);
					}
					if (p_save[x][y + 1] == 254)
					{
						p_save[x][y + 1] = path_length;
						cx.push_back(x);
						cy.push_back(y + 1);
					}
					if (p_save[x + 1][y] == 254)
					{
						p_save[x + 1][y] = path_length;
						cx.push_back(x + 1);
						cy.push_back(y);
					}	
				}
		
				int k;
				if (pp[x][y] == 150) 
				{
					for (k = 0; k < tempx.size(); k++) {
						if (tempy[k] == y && tempx[k] == x)
							break;
					}

					if (k % 2 == 0) 
					{
						x = tempx[k];
						y = tempy[k];
						pp[x][y] = 254; 
						x = tempx[k + 1];
						y = tempy[k + 1];
						pp[x][y] = 254;

						s_listx.push_back(tempx[k]);
						s_listy.push_back(tempy[k]);

						tempx.erase(tempx.begin() + k);
						tempy.erase(tempy.begin() + k);

						s_listx.push_back(tempx[k]);
						s_listy.push_back(tempy[k]);

						tempx.erase(tempx.begin() + k);
						tempy.erase(tempy.begin() + k);
					}
					else 
					{
						x = tempx[k - 1];
						y = tempy[k - 1];

						pp[x][y] = 254;
						x = tempx[k];
						y = tempy[k];
						pp[x][y] = 254;
						s_listx.push_back(tempx[k]);
						s_listy.push_back(tempy[k]);
						tempx.erase(tempx.begin() + k);
						tempy.erase(tempy.begin() + k);
						s_listx.push_back(tempx[k - 1]);
						s_listy.push_back(tempy[k - 1]);
						tempx.erase(tempx.begin() + k - 1);
						tempy.erase(tempy.begin() + k - 1);
					}

					break;
				}

				templistx.clear();
				templisty.clear();
				templistx.resize(cx.size());
				templisty.resize(cy.size());
				templistx.swap(cx);
				templisty.swap(cy);
			}

			PathList path_list=backtrace(endx.back(), endy.back(),p_save,image);

			for(int i=path_list.x.size()-1;i>=0;i--)
			{
				path_tx.push_back(path_list.x[i]);
				path_ty.push_back(path_list.y[i]);

				// circle(image_origin,(Point(path_list.y[i],path_list.x[i])),3,Scalar(200,200,120),-1);
				// imshow("image", image_origin);
				// waitKey(2);

				mypathx[index].push_back((h-path_list.x[i])*0.05+ori_x);
				mypathy[index].push_back(path_list.y[i]*0.05+ori_y);
			}
			// imshow("image", image_origin);
			// waitKey();

			index=index+1;

			endx.clear();
			endy.clear();

			int c;

			for (c = 0; c < checkx.size(); c++) 
			{
				if (checky[c] == s_listy.back() && checkx[c] == s_listx.back())
					break;
			}

			c=c/2;
	
			for (int i=0;i<pathFinding.NumOfPoint[c]-1;i++){

				path_tx.push_back(pathFinding.Path[c][0][i]);
				path_ty.push_back(pathFinding.Path[c][1][i]);

				// circle(image_origin,(Point(pathFinding.Path[c][1][i],pathFinding.Path[c][0][i])),3,Scalar(120.120,120),-1);
				// imshow("image", image_origin);
				// waitKey(2);

				mypathx[index].push_back((h-pathFinding.Path[c][0][i])*0.05+ori_x);
				mypathy[index].push_back(pathFinding.Path[c][1][i]*0.05+ori_y);
			}

			index=index+1;
			// imshow("image", image_origin);
			// waitKey();
		}


		for (int i=0;i<path_tx.size();i++)
		{
			p2.position.y=(h-path_tx[i])*0.05+ori_x;
			p2.position.x=path_ty[i]*0.05+ori_y;
			p2.orientation.w=1.0;
			path_.wall.push_back(p2);

			// circle(image_origin,(Point(path_ty[i],path_tx[i])),3,Scalar(100,150,60),-1);
			// imshow("image", image_origin);
			// waitKey(2);
		}

		for (int i=0;i<index;++i)
		{
			for (int j=0;j<mypathx[i].size();j++)
			{
				temp.position.x=mypathy[i][j];
				temp.position.y=mypathx[i][j];
				temp.orientation.w=1.0;
				P_array.poses.push_back(temp);	
			}
			P_array.header.frame_id="map";
			P_array.header.stamp=ros::Time::now();
			path_.wall_sec.push_back(P_array);
			P_array.poses.clear();
		}

		// imshow("image", image_origin);
		// waitKey();
		ROS_INFO("Path Done");

		return true;
}

// void PathFindingClass::deleteArea(const geometry_msgs::Polygon::ConstPtr& msg)
// {
// 	vector<int> x;
// 	vector<int> y;
// 	for (int i=0;i<msg->points.size();i++)
// 	{
// 		x.push_back(h-(msg->points[i].y-ori_y)/0.05);
// 		y.push_back((msg->points[i].x-ori_x)/0.05);
// 	}

// 		fillPoly(image_origin,ppt,x.size(),1,Scalar(255,255,255),lineType);
// }

void PathFindingClass::drawCallback(const bp_std_msgs::PathAreaConstPtr& msg)
{
	/////////////画区域覆盖/////////////
	Mat image_draw;
	image_draw = image_origin.clone();
	const int h = image_draw.rows;
	const int w = image_draw.cols;
	
	int X0,Y0;
	vector<float> X;
	vector<float> Y;
	int tempx,tempy;

	for(int i=0;i<msg->path_area.poses.size();i++)
	{
		tempy=(msg->path_area.poses[i].position.x-ori_x)/0.05;
		tempx=h-(msg->path_area.poses[i].position.y-ori_y)/0.05;
		X.push_back(tempx);
		Y.push_back(tempy);
	}

	X0 = (X[0]+X[2])*0.5;
	Y0 = (Y[0]+Y[2])*0.5;

	for(int j=0;j<X.size()-1;++j)
			line(image_draw, Point(Y[j],X[j]), Point(Y[j+1],X[j+1]), Scalar(0, 0, 0), 3);
	line(image_draw, Point(Y[3],X[3]), Point(Y[0],X[0]), Scalar(0, 0, 0), 3);

	X.clear();
	Y.clear();
	
	geometry_msgs::Pose p1;

	PathList s_list_rec = bowFinding(image_draw, 0, Y0, X0);
	for (int i = 0; i < s_list_rec.x.size(); i++) 
	{
		Point p(s_list_rec.y[i],s_list_rec.x[i]);
		circle(image_draw,p,5,Scalar(200,60,60),-1);


		p1.position.y=(h-s_list_rec.x[i]-1)*0.05+ori_y; //
		p1.position.x=s_list_rec.y[i]*0.05+ori_x; //
		p1.orientation.w=1.0;
		path_rec.path_area.poses.push_back(p1);
	}
	path_rec.path_area.header.frame_id="map";
	path_rec.path_area.header.stamp=ros::Time::now();
	////////////////////////////////
	path_rec.name_path=msg->name_path;
	path_rec.name_map=msg->name_map;
	////////////////////////////////
	pub_rec.publish(path_rec);
	ROS_INFO("path in rec published");
	
	path_rec.path_area.poses.clear();
	
	// imshow("image",image_draw);
	// waitKey();
}



void PathFindingClass::startPoint(const geometry_msgs::Pose::ConstPtr& msg)  
{
	X = msg->position.x;
	Y = msg->position.y;
	// cout<<"position in world: X="<<X<<" "<<"Y="<<Y<<endl;
}


void PathFindingClass::pathCallback(const bp_std_msgs::AlongWallBowTask::ConstPtr& msg)
{
	 string nameofpath;
	 // char* home;
         // home=getenv("HOME");
	 nameofpath="/home/wl/lslidar_ws/src/bringup/my_map";
	 testMapDir(nameofpath.c_str(),msg->name_map.c_str(),image_origin,ori_x, ori_y);
	 //imshow("image",image_origin);
	 //waitKey(); 

               printf("cp1232: %f\n",ori_x);	

                 flag =	calculatePath(image_origin, ori_y, ori_x, path_);
              ROS_INFO("calculate done");


                    //if (flag)
		   //  { 
		       geometry_msgs::PoseArray P1,P1_bow,P2,P2_wall;
		
			P1.header.frame_id="map";
			P1.header.stamp=ros::Time::now();
			P1_bow.header.frame_id="map";
			P1_bow.header.stamp=ros::Time::now();
			P2.header.frame_id="map";
			P2.header.stamp=ros::Time::now();
			P2_wall.header.frame_id="map";
			P2_wall.header.stamp=ros::Time::now();
			P1.poses=path_.bow;
			P1_bow.poses=path_.bow;
			pub_debug_bow.publish(P1_bow);
			ROS_INFO("path_bow: %lu",path_.bow.size());
			P2.poses=path_.wall;
			P2_wall.poses=path_.wall;
			pub_debug_wall.publish(P2_wall);
			ROS_INFO("path_along_wall: %lu",path_.wall.size());
			P.path_bow=P1;
			// P_a.path_along_wall=P2;
			P.name_map = msg->name_map;
			P.path_along_wall=path_.wall_sec;
			path_.wall_sec.clear();

			// pub_test.publish(test);
			pub_.publish(P);
			P_save_a=P;

			ROS_INFO("Path 1 saved");
                       
                     
     	
				
}


void testMapDir(const char *filepath, string name_map,Mat& image_origin, double& ori_x, double& ori_y)
{


 
  DIR *dir = opendir(filepath); //先打开文件

  if (NULL == dir) //判断是否打开成功
  {
    perror("opendir ");
    return;
  }

  struct dirent *di; //dirent结构体指针，用于指向数据

  char p_file[1024]; //用于拼接字符串。遇到子目录。

  while ((di = readdir(dir)) != NULL)
  {
    //要忽略掉.和 .. 如果读到它们，就不要对他们操作。
    if (strcmp(di->d_name, ".") == 0 || strcmp(di->d_name, "..") == 0)
    {
      continue; //忽略掉
    }
    //遇到目录就要进入，使用递归
    else if (di->d_type == DT_DIR)
    {
      continue; //忽略掉
    }
    else
    {

    //   struct stat st;
      std::string path(filepath);
      path = path + "/" + std::string(di->d_name);
    //   if (stat(path.c_str(), &st) == -1)
    //   {
    //     perror("stat");
    //     return;
    //   }
	
	  
	  std::string name_cp1=di->d_name;
	  std::string name_cp2=name_map;
	  name_cp1=name_cp1.substr(0,name_cp1.length()-5);
	//   name_cp2=name_cp2.substr(0,name_cp2.length()-4);
	//   printf("cp1: %s\n",name_cp1.c_str());
	//   printf("cp2: %s\n",name_cp2.c_str());
	  

      if (path.substr(path.size() - 5) == ".yaml" && name_cp1==name_cp2)
      {
		// printf("yes1\n");
        printf("path = %s\n", path.c_str());

        std::string image;
        double resolution, origin_x, origin_y, origin_z, occupied_thresh, free_thresh;
        int negate;
        if (getMapFileInfo(path, image, resolution, origin_x, origin_y, origin_z, negate, occupied_thresh, free_thresh))
        {
		
          //printf("image = %s,%f,%f,%f,%f,%d,%f,%f\n", image.c_str(), resolution, origin_x, origin_y, origin_z, negate, occupied_thresh, free_thresh);
		  image_origin=imread(image);
		  ori_x=origin_x;
		  ori_y=origin_y;
		}
        else
        {
          printf("getMapFileInfo return false\n");
        }
      }
    }
  }
  closedir(dir);
}

bool getMapFileInfo(std::string fname, std::string &image, double &resolution, double &origin_x, double &origin_y, double &origin_z, int &negate, double &occupied_thresh, double &free_thresh)
{
  std::string mapfname = "";
  double origin[3];
  int neg;
  double occ_th, free_th, res;
  bool trinary = true;

  std::ifstream fin(fname.c_str());
  if (fin.fail())
  {
    ROS_ERROR("Map_server could not open %s.", fname.c_str());
    return 1;
  }
	#ifdef HAVE_NEW_YAMLCPP
  // The document loading process changed in yaml-cpp 0.5.
  		YAML::Node doc = YAML::Load(fin);
	#else
		YAML::Parser parser(fin);
		YAML::Node doc;
		parser.GetNextDocument(doc);
		// printf("ssssssssssssss");
	#endif

	try
	{
		doc["resolution"] >> res;
		resolution = res;
		//  std::string aaa;
		//    doc["origin"].Scalar() >> aaa;
		//    ROS_ERROR("aaa = %s",aaa.c_str());
	}
	catch (YAML::InvalidScalar)
	{
		ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
		return false;
	}

	try
	{

		doc["negate"] >> negate;
		negate = neg;
	}
	catch (YAML::InvalidScalar)
	{
		ROS_ERROR("The map does not contain a negate tag or it is invalid.");
		return false;
	}

	try
	{
		doc["occupied_thresh"] >> occ_th;
		occupied_thresh = occ_th;
	}
	catch (YAML::InvalidScalar)
	{
		ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
		return false;
	}

	try
	{
		doc["free_thresh"] >> free_th;
		free_thresh = free_th;
	}
	catch (YAML::InvalidScalar)
	{
		ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
		return false;
	}

	try
	{
		doc["trinary"] >> trinary;
	}
	catch (YAML::Exception)
	{
		ROS_DEBUG("The map does not contain a trinary tag or it is invalid... assuming true");
		trinary = true;
	}

	try
	{
		doc["origin"][0] >> origin[0];
		doc["origin"][1] >> origin[1];
		doc["origin"][2] >> origin[2];
		origin_x = origin[0];
		origin_y = origin[1];
		origin_z = origin[2];
	}
	catch (YAML::InvalidScalar)
	{
		ROS_ERROR("The map does not contain an origin tag or it is invalid.");
		return false;
	}

	try
	{
		doc["image"] >> mapfname;
		// TODO: make this path-handling more robust
		if (mapfname.size() == 0)
		{
		ROS_ERROR("The image tag cannot be an empty string.");
		return false;
		}
		if (mapfname[0] != '/')
		{
		// dirname can modify what you pass it
		char *fname_copy = strdup(fname.c_str());
		mapfname = std::string(dirname(fname_copy)) + '/' + mapfname;
		free(fname_copy);
		image = mapfname;
		}
	}
	catch (YAML::InvalidScalar)
	{
		ROS_ERROR("The map does not contain an image tag or it is invalid.");
		return false;
	}

  return true;
}

PathList backtrace(int x, int y, int**p, Mat image )
{
	PathList result;
	int minimum;
	int c=0;

	while(p[x][y]!=1000){

		if(p[x-1][y-1]<1000)
		p[x-1][y-1]=50000;
		if(p[x-1][y]<1000)
		p[x-1][y]=50000;
		if(p[x-1][y+1]<1000)
		p[x-1][y+1]=50000;
		if(p[x][y-1]<1000)
		p[x][y-1]=50000;
		if(p[x][y+1]<1000)
		p[x][y+1]=50000;
		if(p[x+1][y-1]<1000)
		p[x+1][y-1]=50000;
		if(p[x+1][y]<1000)
		p[x+1][y]=50000;
		if(p[x+1][y+1]<1000)
		p[x+1][y+1]=50000;

		minimum=p[x-1][y-1];
		minimum<p[x-1][y]?minimum=minimum:minimum=p[x-1][y];
		minimum<p[x-1][y+1]?minimum=minimum:minimum=p[x-1][y+1];
		minimum<p[x][y-1]?minimum=minimum:minimum=p[x][y-1];
		minimum<p[x][y+1]?minimum=minimum:minimum=p[x][y+1];
		minimum<p[x+1][y-1]?minimum=minimum:minimum=p[x+1][y-1];
		minimum<p[x+1][y]?minimum=minimum:minimum=p[x+1][y];
		minimum<p[x+1][y+1]?minimum=minimum:minimum=p[x+1][y+1];

		// minimum=min({p[x-1][y-1],p[x-1][y],p[x-1][y+1],p[x][y-1],p[x][y+1],p[x+1][y-1],p[x+1][y],p[x+1][y+1]});

		if(p[x-1][y-1]==minimum){
			result.x.push_back(x-1);
			result.y.push_back(y-1);
			x=x-1;
			y=y-1;
		}
		else if(p[x-1][y]==minimum){
			result.x.push_back(x-1);
			result.y.push_back(y);
			x=x-1;
		}
		else if(p[x-1][y+1]==minimum){
			result.x.push_back(x-1);
			result.y.push_back(y+1);
			x=x-1;
			y=y+1;
		}
		else if(p[x][y-1]==minimum){
			result.x.push_back(x);
			result.y.push_back(y-1);
			y=y-1;
		}
		else if(p[x][y+1]==minimum){
			result.x.push_back(x);
			result.y.push_back(y+1);
			y=y+1;
		}
		else if(p[x+1][y-1]==minimum){
			result.x.push_back(x+1);
			result.y.push_back(y-1);
			x=x+1;
			y=y-1;
		}
		else if(p[x+1][y]==minimum){
			result.x.push_back(x+1);
			result.y.push_back(y);
			x=x+1;
		}
		else if(p[x+1][y+1]==minimum){
			result.x.push_back(x+1);
			result.y.push_back(y+1);
			x=x+1;
			y=y+1;
		}

		// circle(image,(Point(y,x)),3,Scalar(120.120,120),-1);
		// imshow("image", image);
		// waitKey();
	}

	return result;
}

int** dijkstra(int** p, int y, int x, int h, int w)
{
	int path_flag = 1000;

	vector<int>tempx;
	vector<int>tempy;
	vector<int>cx;
	vector<int>cy;
	vector<int>path;

	tempx.push_back(x); //x0=170
	tempy.push_back(y);  //y0=60
	path.push_back(path_flag);

	while (tempx.empty() != 1)
	{
		cx.clear();
		cy.clear();
		path_flag += 1;

		for (int i = 0; i < tempx.size(); i++)
		{

			if(tempx[i]+K_b>=h||tempy[i]+K_b>=w||tempx[i]-K_b<=0||tempy[i]-K_b<=0)
				return NULL;				

			if (p[tempx[i]][tempy[i] - 1] == 254)
			{
				p[tempx[i]][tempy[i] - 1] = path_flag;
				cx.push_back(tempx[i]);
				cy.push_back(tempy[i] - 1);
				path.push_back(path_flag); 
			}
			if (p[tempx[i] - 1][tempy[i]] == 254)
			{
				p[tempx[i] - 1][tempy[i]] = path_flag;
				cx.push_back(tempx[i] - 1);
				cy.push_back(tempy[i]);
				path.push_back(path_flag);
			}
			if (p[tempx[i]][tempy[i] + 1] == 254)
			{
				p[tempx[i]][tempy[i] + 1] = path_flag;
				cx.push_back(tempx[i]);
				cy.push_back(tempy[i] + 1);
				path.push_back(path_flag);
			}
			if (p[tempx[i] + 1][tempy[i]] == 254)
			{
				p[tempx[i] + 1][tempy[i]] = path_flag;
				cx.push_back(tempx[i] + 1);
				cy.push_back(tempy[i]);
				path.push_back(path_flag);

			}
		}
		
		tempx.clear();
		tempy.clear();
		tempx.resize(cx.size());
		tempy.resize(cy.size());
		tempx.swap(cx);
		tempy.swap(cy);
	}

	
	return p;
}

void dijkstra02(int** p, int y, int x, int h, int w)
{
	int path_flag = 1000;

	vector<int>tempx;
	vector<int>tempy;
	vector<int>cx;
	vector<int>cy;
	vector<int>path;

	tempx.push_back(x); 
	tempy.push_back(y);  
	path.push_back(path_flag);

	while (tempx.empty() != 1)
	{
		cx.clear();
		cy.clear();
		path_flag += 1;

		for (int i = 0; i < tempx.size(); i++)
		{
			if(tempx[i]+K_a>=h||tempy[i]+K_a>=w)
				return;

			if (p[tempx[i]][tempy[i] - 1] == 254)
			{
				p[tempx[i]][tempy[i] - 1] = path_flag;
				cx.push_back(tempx[i]);
				cy.push_back(tempy[i] - 1);
				path.push_back(path_flag); 
			}
			if (p[tempx[i] - 1][tempy[i]] == 254)
			{
				p[tempx[i] - 1][tempy[i]] = path_flag;
				cx.push_back(tempx[i] - 1);
				cy.push_back(tempy[i]);
				path.push_back(path_flag);
			}
			if (p[tempx[i]][tempy[i] + 1] == 254)
			{
				p[tempx[i]][tempy[i] + 1] = path_flag;
				cx.push_back(tempx[i]);
				cy.push_back(tempy[i] + 1);
				path.push_back(path_flag);
			}
			if (p[tempx[i] + 1][tempy[i]] == 254)
			{
				p[tempx[i] + 1][tempy[i]] = path_flag;
				cx.push_back(tempx[i] + 1);
				cy.push_back(tempy[i]);
				path.push_back(path_flag);
			}
		}

		tempx.clear();
		tempy.clear();
		tempx.resize(cx.size());
		tempy.resize(cy.size());
		tempx.swap(cx);
		tempy.swap(cy);
	}
}

Edgeway edgeway_path(int** img, int h, int w, int x, int y)
{
	vector<int>aw_x;
	vector<int>aw_y;
	int y0 = 0, x0 = 0;
	int i, j;
	const int B = 0, W = 240;

	int** aw = new int* [2];
	Edgeway result;

	aw_x.push_back(x);
	aw_y.push_back(y);
	img[x][y] = 150;

	if (img[x][y - 1] == B && img[x - 1][y - 1] == W && img[x - 1][y] == W) {
		x -= 1; y -= 1;
	} 
	else if (img[x - 1][y - 1] == B && img[x - 1][y] == W) {
		x -= 1;
	} 
	else if (img[x - 1][y] == B && img[x - 1][y + 1] == W && img[x][y + 1] == W) {
		x -= 1; y += 1;
	} 
	else if (img[x - 1][y + 1] == B && img[x][y + 1] == W) {
		y += 1;
	} 
	else if (img[x][y + 1] == B && img[x + 1][y + 1] == W && img[x + 1][y] == W) {
		x += 1; y += 1;
	} 
	else if (img[x + 1][y + 1] == B && img[x + 1][y] == W) {
		x += 1;
	} 
	else if (img[x + 1][y] == B && img[x + 1][y - 1] == W && img[x][y - 1] == W) {
		x += 1; y -= 1;
	} 
	else if (img[x + 1][y - 1] == B && img[x][y - 1] == W) {
		y -= 1;
	} 

	else if (img[x][y - 1] == B && img[x + 1][y - 1] == W && img[x + 1][y] == W) {
		x += 1; y -= 1;
	} 
	else if (img[x + 1][y - 1] == B && img[x + 1][y] == W) {
		x += 1;
	} 
	else if (img[x + 1][y] == B && img[x + 1][y + 1] == W && img[x][y + 1] == W) {
		x += 1; y += 1;
	} 
	else if (img[x + 1][y + 1] == B && img[x][y + 1] == W) {
		y += 1;
	} 
	else if (img[x][y + 1] == B && img[x - 1][y + 1] == W && img[x - 1][y] == W) {
		x -= 1; y += 1;
	} 
	else if (img[x - 1][y + 1] == B && img[x - 1][y] == W) {
		x -= 1;
	} 
	else if (img[x - 1][y] == B && img[x - 1][y - 1] == W && img[x][y - 1] == W) {
		x -= 1; y -= 1;
	} 
	else if (img[x - 1][y - 1] == B && img[x][y - 1] == W) {
		y -= 1;
	}

	aw_x.push_back(x);
	aw_y.push_back(y);
	img[x][y] = 150;

	while (aw_x[0] != aw_x.back() || aw_y[0] != aw_y.back())
	{
		if (img[x][y - 1] == B && img[x - 1][y - 1] == W && img[x - 1][y] != B) {
			x -= 1; y -= 1;
		} //4ºÚ1°×2°××ß1Î»
		else if (img[x - 1][y - 1] == B && img[x - 1][y] == W) {
			x -= 1;
		} //1ºÚ2°××ß2Î»
		else if (img[x - 1][y] == B && img[x - 1][y + 1] == W && img[x][y + 1] != B) {
			x -= 1; y += 1;
		} //2ºÚ3°×6°××ß3Î»
		else if (img[x - 1][y + 1] == B && img[x][y + 1] == W) {
			y += 1;
		} //3ºÚ6°××ß6Î»
		else if (img[x][y + 1] == B && img[x + 1][y + 1] == W && img[x + 1][y] != B) {
			x += 1; y += 1;
		} //6ºÚ9°×8°××ß9Î»
		else if (img[x + 1][y + 1] == B && img[x + 1][y] == W) {
			x += 1;
		} //9ºÚ8°××ß8Î»
		else if (img[x + 1][y] == B && img[x + 1][y - 1] == W && img[x][y - 1] != B) {
			x += 1; y -= 1;
		} //8ºÚ7°×4°××ß7Î»
		else if (img[x + 1][y - 1] == B && img[x][y - 1] == W) {
			y -= 1;
		} //7ºÚ4°××ß4Î»

		//ÄæÊ±Õë
		else if (img[x][y - 1] == B && img[x + 1][y - 1] == W && img[x + 1][y] != B) {
			x += 1; y -= 1;
		} //4ºÚ7°×8°××ß7Î»
		else if (img[x + 1][y - 1] == B && img[x + 1][y] == W) {
			x += 1;
		} //7ºÚ8°××ß8Î»
		else if (img[x + 1][y] == B && img[x + 1][y + 1] == W && img[x][y + 1] != B) {
			x += 1; y += 1;
		} //8ºÚ9°×6°××ß9Î»
		else if (img[x + 1][y + 1] == B && img[x][y + 1] == W) {
			y += 1;
		} //9ºÚ6°××ß6Î»
		else if (img[x][y + 1] == B && img[x - 1][y + 1] == W && img[x - 1][y] != B) {
			x -= 1; y += 1;
		} //6ºÚ2°×3°××ß3Î»
		else if (img[x - 1][y + 1] == B && img[x - 1][y] == W) {
			x -= 1;
		} //3ºÚ2°××ß2Î»
		else if (img[x - 1][y] == B && img[x - 1][y - 1] == W && img[x][y - 1] != B) {
			x -= 1; y -= 1;
		} //2ºÚ1°×4°××ß1Î»
		else if (img[x - 1][y - 1] == B && img[x][y - 1] == W) {
			y -= 1;
		}
		else { //ËÄÖÜ¶¼Ã»ÓÐ°×É«¸ñ×Ó£¬ÑØÔ­Â··µ»Ø£¬ÕÒ×î½üµÄ°×É«¸ñ×Ó
			int l = aw_x.size();
			for (i = 0; i < l; i++) {
				y = aw_y[l - i - 1];
				x = aw_x[l - i - 1];
				if (img[x - 1][y - 1] == w || img[x - 1][y] == w || img[x - 1][y + 1] == w || img[x][y - 1] == w || img[x][y + 1] == w || img[x + 1][y - 1] == w || img[x + 1][y] == w || img[x + 1][y + 1] == w) {
					if (x0 == x && y0 == y) {
						for (i = 0; i < 2; i++)
							aw[i] = new int[aw_x.size()];
						for (j = 0; j < aw_x.size(); j++)
						{
							aw[0][j] = aw_x[j];
							aw[1][j] = aw_y[j];
						}
						result.edgeway = aw;
						result.num = aw_x.size();
						return result;
					}
					x0 = x; y0 = y;
					break;
				}
			}
		}
		aw_x.push_back(x);
		aw_y.push_back(y);
		img[x][y] = 150;
	}

	for (i = 0; i < 2; i++)
		aw[i] = new int[aw_x.size()];
	for (j = 0; j < aw_x.size(); j++)
	{
		aw[0][j] = aw_x[j];
		aw[1][j] = aw_y[j];
	}
	result.edgeway = aw;
	result.num = aw_x.size();
	return result;
}

WallPath dijAlong(int** pr, int h, int w, int x, int y)
{
	int i, j;

	int** img = new int* [h];
	for (i = 0; i < h; i++)
		img[i] = new int[w];
	for (i = 0; i < h; i++)
	{
		for (j = 0; j < w; j++)
		{
			if (pr[i][j] == 240)
				img[i][j] = 240;
			else
				img[i][j] = 0;
		}
	}

	vector<int>tempx;
	vector<int>tempy;
	vector<int>cx;
	vector<int>cy;
	tempx.push_back(x);
	tempy.push_back(y);
	int ty, tx, c = 0; //c is number of path
	int path_flag = 1000;

	WallPath result;


	const int num = preset_num; 
	int*** wallPath = new int** [num];
	for (i = 0; i < num; i++)
		wallPath[i] = new int* [2];

	while (tempx.empty() != 1)
	{
		
		path_flag += 1;
		cx.clear();
		cy.clear();
		for (i = 0; i < tempx.size(); i++)
		{

			if (pr[tempx[i]][tempy[i]] == 150)
				continue;

			int* p = new int(0);

			if (pr[tempx[i]][tempy[i] - 1] == 240) {
				pr[tempx[i]][tempy[i] - 1] = path_flag;
				cx.push_back(tempx[i]);
				cy.push_back(tempy[i] - 1);
			}
			//////////////////////////////////////////////////////////////////
			else if (pr[tempx[i]][tempy[i] - 1] == 0 && *p == 0) {

				Edgeway alongWall = edgeway_path(img, h, w, tempx[i], tempy[i]);
				c += 1;
				*p = *p + 1;
				for (int m = 0; m < 2; m++) {
					wallPath[c - 1][m] = new int[alongWall.num];
				}
				for (int n = 0; n < alongWall.num; n++) {
					wallPath[c - 1][0][n] = alongWall.edgeway[0][n];
					wallPath[c - 1][1][n] = alongWall.edgeway[1][n];
					tx = alongWall.edgeway[0][n];
					ty = alongWall.edgeway[1][n];
					pr[tx][ty] = 150;
				}
				delete[]alongWall.edgeway;
				result.NumOfPoint.push_back(alongWall.num);
			}
			///////////////////////////////////////////////////////////////////
			if (pr[tempx[i] - 1][tempy[i]] == 240) {
				pr[tempx[i] - 1][tempy[i]] = path_flag;
				cx.push_back(tempx[i] - 1);
				cy.push_back(tempy[i]);
			}
			///////////////////////////////////////////////////////////////////
			else if (pr[tempx[i] - 1][tempy[i]] == 0 && *p == 0) {


				

				Edgeway alongWall = edgeway_path(img, h, w, tempx[i], tempy[i]);
				c += 1;
				*p = *p + 1;
				for (int m = 0; m < 2; m++) {
					wallPath[c - 1][m] = new int[alongWall.num];
				}
				for (int n = 0; n < alongWall.num; n++) {
					wallPath[c - 1][0][n] = alongWall.edgeway[0][n];
					wallPath[c - 1][1][n] = alongWall.edgeway[1][n];
					tx = alongWall.edgeway[0][n];
					ty = alongWall.edgeway[1][n];
					pr[tx][ty] = 150;
				}
				delete[]alongWall.edgeway;
				result.NumOfPoint.push_back(alongWall.num);
			}
			////////////////////////////////////////////////////////////////////

			if (pr[tempx[i]][tempy[i] + 1] == 240) {
				pr[tempx[i]][tempy[i] + 1] = path_flag;
				cx.push_back(tempx[i]);
				cy.push_back(tempy[i] + 1);
			}
			
			////////////////////////////////////////////////////////////////////
			else if (pr[tempx[i]][tempy[i] + 1] == 0 && *p == 0) {


				Edgeway alongWall = edgeway_path(img, h, w, tempx[i], tempy[i]);
				c += 1;
				*p = *p + 1;
				for (int m = 0; m < 2; m++) {
					wallPath[c - 1][m] = new int[alongWall.num];
				}
				for (int n = 0; n < alongWall.num; n++) {
					wallPath[c - 1][0][n] = alongWall.edgeway[0][n];
					wallPath[c - 1][1][n] = alongWall.edgeway[1][n];
					tx = alongWall.edgeway[0][n];
					ty = alongWall.edgeway[1][n];
					pr[tx][ty] = 150;
				}
				delete[]alongWall.edgeway;
				result.NumOfPoint.push_back(alongWall.num);
			}
			/////////////////////////////////////////////////////////////////////
			if (pr[tempx[i] + 1][tempy[i]] == 240) {
				pr[tempx[i] + 1][tempy[i]] = path_flag;
				cx.push_back(tempx[i] + 1);
				cy.push_back(tempy[i]);
			}
			/////////////////////////////////////////////////////////////////////
			else if (pr[tempx[i] + 1][tempy[i]] == 0 && *p == 0) {


				Edgeway alongWall = edgeway_path(img, h, w, tempx[i], tempy[i]);
				c += 1;
				*p = *p + 1;
				for (int m = 0; m < 2; m++) {
					wallPath[c - 1][m] = new int[alongWall.num];
				}
				for (int n = 0; n < alongWall.num; n++) {
					wallPath[c - 1][0][n] = alongWall.edgeway[0][n];
					wallPath[c - 1][1][n] = alongWall.edgeway[1][n];
					tx = alongWall.edgeway[0][n];
					ty = alongWall.edgeway[1][n];
					pr[tx][ty] = 150;
				}
				delete[]alongWall.edgeway;
				result.NumOfPoint.push_back(alongWall.num);
			}

			/////////////////////////////////////////////////////////////////////
			delete p;
		}
		tempx.clear();
		tempy.clear();
		tempx.resize(cx.size());
		tempy.resize(cy.size());
		tempx.swap(cx);
		tempy.swap(cy);
	}
	delete[]img;

	result.Path = wallPath;
	result.Num = c;

	return result;
}

PathList maxLengthPath_w(int MinLeng, int** pr, int h, int w)
{

	int addfront = 0, add, step10;
	WPList maxWaypointList;
	for (step10 = 0; step10 < 10; step10++) {
		add = 0;
//ROS_ERROR("?!!!");
		WPList tempList = getWaypointList_w(step10, pr, h, w);
//ROS_ERROR("13243?!!!");
		for (int i = 0; i < tempList.way_x[0].size(); i++) {
			add += tempList.way_x[1][i];

		}
		
		if (add > addfront) {
			addfront = add;
			maxWaypointList = tempList;
//ROS_ERROR("weerrrrt?!!!");
		}
	}

	vector<int>filterMinLeng_x;
	vector<int>filterMinLeng_y;
	if (maxWaypointList.way_x.size() != 0) {
		for (int i = 0; i < maxWaypointList.way_x[0].size(); i++) {
			if (maxWaypointList.way_x[1][i] > MinLeng) {
				filterMinLeng_x.push_back(maxWaypointList.way_x[0][i - 1]);
				filterMinLeng_x.push_back(maxWaypointList.way_x[0][i]);
				filterMinLeng_y.push_back(maxWaypointList.way_y[0][i - 1]);
				filterMinLeng_y.push_back(maxWaypointList.way_y[0][i]);
			}
		}
	}

	PathList result;
	result.x = filterMinLeng_x;
	result.y = filterMinLeng_y;
	return result;
}

PathList maxLengthPath_h(int MinLeng, int** pr, int h, int w)
{
	int addfront = 0, add, step10;
	WPList maxWaypointList;
	for (step10 = 0; step10 < 10; step10++) {
		add = 0;
		WPList tempList = getWaypointList_h(step10, pr, h, w);
		for (int i = 0; i < tempList.way_x[0].size(); i++) {
			add += tempList.way_x[1][i];
		}
		if (add > addfront) {
			addfront = add;
			maxWaypointList = tempList;
		}
	}
	vector<int>filterMinLeng_x;
	vector<int>filterMinLeng_y;
	if (maxWaypointList.way_x.size() != 0) {
		for (int i = 0; i < maxWaypointList.way_x[0].size(); i++) {
			if (maxWaypointList.way_x[1][i] > MinLeng) {
				filterMinLeng_x.push_back(maxWaypointList.way_x[0][i - 1]);
				filterMinLeng_x.push_back(maxWaypointList.way_x[0][i]);
				filterMinLeng_y.push_back(maxWaypointList.way_y[0][i - 1]);
				filterMinLeng_y.push_back(maxWaypointList.way_y[0][i]);
			}
		}
	}
	PathList result;
	result.x = filterMinLeng_x;
	result.y = filterMinLeng_y;
	return result;
}

WPList getWaypointList_w(int step10, int** pr, int h, int w)
{
	
	//int i, j, n, r = 10;
	int i, j, n, r = 5;
	int map254, length;
	vector<int>k;
	vector<vector<int> >way_x(2);
	vector<vector<int> >way_y(2);

	for (n = 0; n < h / interval; n++) {
		k.push_back(interval * n + step10);
	}

	for (i = 0; i < k.size(); i++) {
		map254 = 0;
		length = 0;

//cout<<"asadfsdfd:"<<r<<endl;

		for (j = r; j < (w - r); j++) {

			if (pr[k[i]][j] != 254)
			{
				map254 = 0;
				continue;
			}
			if (map254 == 0)
			{
				length = 1;
				map254 = 1;
				way_x[0].push_back(j);
				way_y[0].push_back(k[i]);
				way_x[0].push_back(j);
				way_y[0].push_back(k[i]);
				way_x[1].push_back(length);
				way_y[1].push_back(length);
				way_x[1].push_back(length);
				way_y[1].push_back(length);
			}
			else
			{
				way_x[0].pop_back();
				way_y[0].pop_back();
				way_x[1].pop_back();
				way_y[1].pop_back();
				length += 1;
				way_x[0].push_back(j);
				way_y[0].push_back(k[i]);
				way_x[1].push_back(length);
				way_y[1].push_back(length);
			}
		}
	}

	WPList result;
	result.way_x = way_x;
	result.way_y = way_y;
	return result;
}
WPList getWaypointList_h(int step10, int** pr, int h, int w)
{
	
	//int i, j, n, r = 10;
	int i, j, n, r = 5;
	int map254, length;
	vector<int>k;
	vector<vector<int> >way_x(2);
	vector<vector<int> >way_y(2);
	for (n = 0; n < w / interval; n++) {
		k.push_back(interval * n + step10);
	}
	for (i = 0; i < k.size(); i++) {
		map254 = 0;
		length = 0;
		for (j = r; j < h - r; j++) {
			if (pr[j][k[i]] != 254)
			{
				map254 = 0;
				continue;
			}
			if (map254 == 0)
			{
				length = 1;
				map254 = 1;
				way_x[0].push_back(k[i]);
				way_y[0].push_back(j);
				way_x[0].push_back(k[i]);
				way_y[0].push_back(j);
				way_x[1].push_back(length);
				way_y[1].push_back(length);
				way_x[1].push_back(length);
				way_y[1].push_back(length);
			}
			else
			{
				way_x[0].pop_back();
				way_y[0].pop_back();
				way_x[1].pop_back();
				way_y[1].pop_back();
				length += 1;
				way_x[0].push_back(k[i]);
				way_y[0].push_back(j);
				way_x[1].push_back(length);
				way_y[1].push_back(length);
			}
		}
	}
	WPList result;
	result.way_x = way_x;
	result.way_y = way_y;
	return result;
}

PathList pathSorted(PathList getWaypointList, int** p, Mat erode_img_save)
{
    int i,j;

	const int h = erode_img_save.rows;
	const int w = erode_img_save.cols;

	vector<int>tempx;
	vector<int>tempy;
	vector<int>s_listx;
	vector<int>s_listy;

	tempx = getWaypointList.x;
	tempy = getWaypointList.y;
	p[tempx[0]][tempy[0]] = 254;

	s_listx.push_back(tempx[0]);
	s_listy.push_back(tempy[0]);

	tempx.erase(tempx.begin());
	tempy.erase(tempy.begin());

	p[tempx[0]][tempy[0]] = 254;

	s_listx.push_back(tempx[0]);
	s_listy.push_back(tempy[0]);

	tempx.erase(tempx.begin());
	tempy.erase(tempy.begin());

	vector<int>cx;
	vector<int>cy;
	vector<int>templistx;
	vector<int>templisty;
	int x, y;
	int** p_save = new int* [h];
	for (i = 0; i < h; i++)
		p_save[i] = new int[w];
	int path_length;
	while (tempx.empty() != 1)
	{
		x = s_listx.back();
		y = s_listy.back();

		path_length = 1000;

		for (i = 0; i < h; i++)
		{
			for (j = 0; j < w; j++)
			{
				p_save[i][j] = erode_img_save.at<Vec3b>(i, j)[0];

			}
		}
		p_save[x][y] = path_length;
		templistx.clear();
		templisty.clear(); 
		templistx.push_back(x);
		templisty.push_back(y);

		while (templistx.empty() != 1) {
			cx.clear();
			cy.clear();
			path_length += 1;
			for (i = 0; i < templistx.size(); i++) {
				x = templistx[i];
				y = templisty[i];

				if (p[x][y] == 150)
					break;
				if (p_save[x][y - 1] == 254)
				{
					p_save[x][y - 1] = path_length;
					cx.push_back(x);
					cy.push_back(y - 1);
				}
				if (p_save[x - 1][y] == 254)
				{
					p_save[x - 1][y] = path_length;
					cx.push_back(x - 1);
					cy.push_back(y);
				}
				if (p_save[x][y + 1] == 254)
				{
					p_save[x][y + 1] = path_length;
					cx.push_back(x);
					cy.push_back(y + 1);
				}
				if (p_save[x + 1][y] == 254)
				{
					p_save[x + 1][y] = path_length;
					cx.push_back(x + 1);
					cy.push_back(y);
				}
			}

			int k;
			if (p[x][y] == 150) {
				for (k = 0; k < tempx.size(); k++) {
					if (tempy[k] == y && tempx[k] == x)
						break;
				}
				if (k % 2 == 0) {
					x = tempx[k];
					y = tempy[k];
					p[x][y] = 254; 
					x = tempx[k + 1];
					y = tempy[k + 1];
					p[x][y] = 254;

					s_listx.push_back(tempx[k]);
					s_listy.push_back(tempy[k]);

					tempx.erase(tempx.begin() + k);
					tempy.erase(tempy.begin() + k);

					s_listx.push_back(tempx[k]);
					s_listy.push_back(tempy[k]);

					tempx.erase(tempx.begin() + k);
					tempy.erase(tempy.begin() + k);

				}
				else {
					x = tempx[k - 1];
					y = tempy[k - 1];

					p[x][y] = 254;
					x = tempx[k];
					y = tempy[k];
					p[x][y] = 254;
					s_listx.push_back(tempx[k]);
					s_listy.push_back(tempy[k]);
					tempx.erase(tempx.begin() + k);
					tempy.erase(tempy.begin() + k);
					s_listx.push_back(tempx[k - 1]);
					s_listy.push_back(tempy[k - 1]);
					tempx.erase(tempx.begin() + k - 1);
					tempy.erase(tempy.begin() + k - 1);
				}
				break;
			}

			templistx.clear();
			templisty.clear();
			templistx.resize(cx.size());
			templisty.resize(cy.size());
			templistx.swap(cx);
			templisty.swap(cy);
		}
	}

	PathList result;
	result.x=s_listx;
	result.y=s_listy;
	return result;

}

int length(PathList len)
{
	int x0,x1,y0,y1,xd,yd;
	int result=0;
        //cout<<"x0:"<<x1<<endl;
        //cout<<"y0:"<<y1<<endl;

	for (int i=0;i<len.x.size()/2;i++)
	{
		x0=len.x[i*2];
		x1=len.x[i*2+1];
		y0=len.y[i*2];
		y1=len.y[i*2+1];
		xd=x1-x0;
		yd=y1-y0;
		result=result+sqrt(pow(xd,2)+pow(yd,2));

             // cout<<"yd:"<<yd<<endl; 
          
	}
	return result;
}

PathList pointFinding(Mat erode_img, int**p, int** pr)
{
	vector<vector<int> >getWaypointList(2);
	int x0, y0, x1, y1;
	int i,j;
	const int h = erode_img.rows;
	const int w = erode_img.cols;


/*
ROS_ERROR("21616483642783werwrew???????????!!!");
	////////////////////////////////////////////////////////////////////////////add
	PathList availablePathList9 = maxLengthPath_w(500, pr, h, w);
ROS_ERROR("2161648364278367326????????????????!!!");
	for (i = 0; i < availablePathList9.x.size(); i++) {
		p[availablePathList9.y[i]][availablePathList9.x[i]] = 150;
		getWaypointList[0].push_back(availablePathList9.x[i]);
		getWaypointList[1].push_back(availablePathList9.y[i]);
	}
	for (i = 0; i < availablePathList9.x.size() / 2; i++) {
		x0 = availablePathList9.x[i * 2];
		y0 = availablePathList9.y[i * 2];
		x1 = availablePathList9.x[i * 2 + 1];
		y1 = availablePathList9.y[i * 2 + 1];
		line(erode_img, Point(x0, y0), Point(x1, y1), Scalar(0, 0, 0), 20);
	}
	for (i = 0; i < h; i++)
	{
		for (j = 0; j < w; j++)
		{
			pr[i][j] = erode_img.at<Vec3b>(i, j)[0];
		}
	}
	/////////////////////////////////////////////////////////////////////////////add
	PathList availablePathList10 = maxLengthPath_h(500, pr, h, w); 
	for (i = 0; i < availablePathList10.x.size(); i++) {
		p[availablePathList10.y[i]][availablePathList10.x[i]] = 150;
		getWaypointList[0].push_back(availablePathList10.x[i]);
		getWaypointList[1].push_back(availablePathList10.y[i]);
	}
	for (i = 0; i < availablePathList10.x.size() / 2; i++) {
		x0 = availablePathList10.x[i * 2];
		y0 = availablePathList10.y[i * 2];
		x1 = availablePathList10.x[i * 2 + 1];
		y1 = availablePathList10.y[i * 2 + 1];
		line(erode_img, Point(x0, y0), Point(x1, y1), Scalar(0, 0, 0), 20);
	}
	for (i = 0; i < h; i++)
	{
		for (j = 0; j < w; j++)
		{
			pr[i][j] = erode_img.at<Vec3b>(i, j)[0];
		}
	}

*/

	PathList availablePathList1 = maxLengthPath_w(100, pr, h, w);

	for (i = 0; i < availablePathList1.x.size(); i++) {
		p[availablePathList1.y[i]][availablePathList1.x[i]] = 150;
		getWaypointList[0].push_back(availablePathList1.x[i]);
		getWaypointList[1].push_back(availablePathList1.y[i]);
	}

	for (i = 0; i < availablePathList1.x.size() / 2; i++) {
		x0 = availablePathList1.x[i * 2];
		y0 = availablePathList1.y[i * 2];
		x1 = availablePathList1.x[i * 2 + 1];
		y1 = availablePathList1.y[i * 2 + 1];
		line(erode_img, Point(x0, y0), Point(x1, y1), Scalar(0, 0, 0), 20);
	}
	for (i = 0; i < h; i++)
	{
		for (j = 0; j < w; j++)
		{
			pr[i][j] = erode_img.at<Vec3b>(i, j)[0];
		}
	}

	/////////////////////////////////////////////////////////////////////////////////


	/////////////////////////////////////////////////////////////////////////////////
	PathList availablePathList2 = maxLengthPath_h(100, pr, h, w);
	for (i = 0; i < availablePathList2.x.size(); i++) {
		p[availablePathList2.y[i]][availablePathList2.x[i]] = 150;
		getWaypointList[0].push_back(availablePathList2.x[i]);
		getWaypointList[1].push_back(availablePathList2.y[i]);
	}
	for (i = 0; i < availablePathList2.x.size() / 2; i++) {
		x0 = availablePathList2.x[i * 2];
		y0 = availablePathList2.y[i * 2];
		x1 = availablePathList2.x[i * 2 + 1];
		y1 = availablePathList2.y[i * 2 + 1];
		line(erode_img, Point(x0, y0), Point(x1, y1), Scalar(0, 0, 0), 20);
	}
	for (i = 0; i < h; i++)
	{
		for (j = 0; j < w; j++)
		{
			pr[i][j] = erode_img.at<Vec3b>(i, j)[0];
		}
	}
	//////////////////////////////////////////////////////////////////////////////////


	//////////////////////////////////////////////////////////////////////////////////
	PathList availablePathList3 = maxLengthPath_w(60, pr, h, w);
	for (i = 0; i < availablePathList3.x.size(); i++) {
		p[availablePathList3.y[i]][availablePathList3.x[i]] = 150;
		getWaypointList[0].push_back(availablePathList3.x[i]);
		getWaypointList[1].push_back(availablePathList3.y[i]);
	}
	for (i = 0; i < availablePathList3.x.size() / 2; i++) {
		x0 = availablePathList3.x[i * 2];
		y0 = availablePathList3.y[i * 2];
		x1 = availablePathList3.x[i * 2 + 1];
		y1 = availablePathList3.y[i * 2 + 1];
		line(erode_img, Point(x0, y0), Point(x1, y1), Scalar(0, 0, 0), 20);
	}
	for (i = 0; i < h; i++)
	{
		for (j = 0; j < w; j++)
		{
			pr[i][j] = erode_img.at<Vec3b>(i, j)[0];
		}
	}
	//////////////////////////////////////////////////////////////////////////////
	PathList availablePathList4 = maxLengthPath_h(60, pr, h, w);
	for (i = 0; i < availablePathList4.x.size(); i++) {
		p[availablePathList4.y[i]][availablePathList4.x[i]] = 150;
		getWaypointList[0].push_back(availablePathList4.x[i]);
		getWaypointList[1].push_back(availablePathList4.y[i]);
	}
	for (i = 0; i < availablePathList4.x.size() / 2; i++) {
		x0 = availablePathList4.x[i * 2];
		y0 = availablePathList4.y[i * 2];
		x1 = availablePathList4.x[i * 2 + 1];
		y1 = availablePathList4.y[i * 2 + 1];
		line(erode_img, Point(x0, y0), Point(x1, y1), Scalar(0, 0, 0), 20);
	}
	for (i = 0; i < h; i++)
	{
		for (j = 0; j < w; j++)
		{
			pr[i][j] = erode_img.at<Vec3b>(i, j)[0];
		}
	}
	////////////////////////////////////////////////////////////////////////////
	PathList availablePathList5 = maxLengthPath_w(40, pr, h, w);
	for (i = 0; i < availablePathList5.x.size(); i++) {
		p[availablePathList5.y[i]][availablePathList5.x[i]] = 150;
		getWaypointList[0].push_back(availablePathList5.x[i]);
		getWaypointList[1].push_back(availablePathList5.y[i]);
	}
	for (i = 0; i < availablePathList5.x.size() / 2; i++) {
		x0 = availablePathList5.x[i * 2];
		y0 = availablePathList5.y[i * 2];
		x1 = availablePathList5.x[i * 2 + 1];
		y1 = availablePathList5.y[i * 2 + 1];
		line(erode_img, Point(x0, y0), Point(x1, y1), Scalar(0, 0, 0), 20);
	}
	for (i = 0; i < h; i++)
	{
		for (j = 0; j < w; j++)
		{
			pr[i][j] = erode_img.at<Vec3b>(i, j)[0];
		}
	}
	/////////////////////////////////////////////////////////////////////////////
	PathList availablePathList6 = maxLengthPath_h(40, pr, h, w); 
	for (i = 0; i < availablePathList6.x.size(); i++) {
		p[availablePathList6.y[i]][availablePathList6.x[i]] = 150;
		getWaypointList[0].push_back(availablePathList6.x[i]);
		getWaypointList[1].push_back(availablePathList6.y[i]);
	}
	for (i = 0; i < availablePathList6.x.size() / 2; i++) {
		x0 = availablePathList6.x[i * 2];
		y0 = availablePathList6.y[i * 2];
		x1 = availablePathList6.x[i * 2 + 1];
		y1 = availablePathList6.y[i * 2 + 1];
		line(erode_img, Point(x0, y0), Point(x1, y1), Scalar(0, 0, 0), 20);
	}
	for (i = 0; i < h; i++)
	{
		for (j = 0; j < w; j++)
		{
			pr[i][j] = erode_img.at<Vec3b>(i, j)[0];
		}
	}

/*
	////////////////////////////////////////////////////////////////////////////add
	PathList availablePathList7 = maxLengthPath_w(20, pr, h, w);
	for (i = 0; i < availablePathList7.x.size(); i++) {
		p[availablePathList5.y[i]][availablePathList7.x[i]] = 150;
		getWaypointList[0].push_back(availablePathList7.x[i]);
		getWaypointList[1].push_back(availablePathList7.y[i]);
	}
	for (i = 0; i < availablePathList7.x.size() / 2; i++) {
		x0 = availablePathList7.x[i * 2];
		y0 = availablePathList7.y[i * 2];
		x1 = availablePathList7.x[i * 2 + 1];
		y1 = availablePathList7.y[i * 2 + 1];
		line(erode_img, Point(x0, y0), Point(x1, y1), Scalar(0, 0, 0), 20);
	}
	for (i = 0; i < h; i++)
	{
		for (j = 0; j < w; j++)
		{
			pr[i][j] = erode_img.at<Vec3b>(i, j)[0];
		}
	}
	/////////////////////////////////////////////////////////////////////////////add
	PathList availablePathList8 = maxLengthPath_h(20, pr, h, w); 
	for (i = 0; i < availablePathList8.x.size(); i++) {
		p[availablePathList8.y[i]][availablePathList8.x[i]] = 150;
		getWaypointList[0].push_back(availablePathList8.x[i]);
		getWaypointList[1].push_back(availablePathList8.y[i]);
	}
	for (i = 0; i < availablePathList8.x.size() / 2; i++) {
		x0 = availablePathList8.x[i * 2];
		y0 = availablePathList8.y[i * 2];
		x1 = availablePathList8.x[i * 2 + 1];
		y1 = availablePathList8.y[i * 2 + 1];
		line(erode_img, Point(x0, y0), Point(x1, y1), Scalar(0, 0, 0), 20);
	}
	for (i = 0; i < h; i++)
	{
		for (j = 0; j < w; j++)
		{
			pr[i][j] = erode_img.at<Vec3b>(i, j)[0];
		}
	}

*/

/////////////////////////////////////////////////////////////////////////////

	PathList result;
	result.x=getWaypointList[1];
	result.y=getWaypointList[0];
	return result;
}

newAfterRotation rotation(int y,int x,double angle,int h, int w)
{
	double A = cos((angle/180.0)*M_PI);
	double B = sin((angle/180.0)*M_PI);

	int pX,pY;
	pX=(x-h/2)*B+(y-w/2)*A+w/2;
	pY=(x-h/2)*A-(y-w/2)*B+h/2;

	newAfterRotation result;
	result.x=pX;
	result.y=pY;

	return result;
}

PathList bowFinding(Mat image_origin, double angle, int Y_, int X_)
{
	PathList null;
	null.x.push_back(0);
	null.y.push_back(0);

	const int h = image_origin.rows;
	const int w = image_origin.cols;
	int i,j;

	Mat rot_mat, img, erode_element, erode_img;
	Mat ima=image_origin.clone();
	Point2f center(ima.cols/2, ima.rows/2);
	rot_mat = getRotationMatrix2D(center, -angle, 1);
    Scalar borderColor = Scalar(205, 205, 205);
	warpAffine(ima, img, rot_mat, ima.size(), INTER_LINEAR, BORDER_CONSTANT, borderColor);
	erode_element = getStructuringElement(MORPH_ELLIPSE, Size(K_b, K_b));
	erode(img, erode_img, erode_element);

	int** pr = new int* [h];  //判断h > 0 ????
	for (i = 0; i < h; i++)
	pr[i] = new int[w];
	for (i = 0; i < h; i++)
	{
		for (j = 0; j < w; j++)
		{
			pr[i][j] = erode_img.at<Vec3b>(i, j)[0];
			if (pr[i][j] <= 255 && pr[i][j]>200)
			{
				pr[i][j] = 254;
			}
			else
				pr[i][j] = 0;
		}
	}


	newAfterRotation dji=rotation(Y_,X_,-angle,h,w);

	int** p_di = dijkstra(pr, dji.x, dji.y,h,w);


	if(p_di==NULL)
		return null;

	int** p = new int* [h]; 
	for (i = 0; i < h; i++)
		p[i] = new int[w];

	
	int check=0;

	for (i = 0; i < h; i++)
	{
		for (j = 0; j < w; j++)
		{
			if (p_di[i][j] >= 1000)
			{
				erode_img.at<Vec3b>(i, j)[0] = 254;
				erode_img.at<Vec3b>(i, j)[1] = 254;
				erode_img.at<Vec3b>(i, j)[2] = 254;
				pr[i][j] = 254;
				p[i][j] = 254;
			}
			else
			{
				erode_img.at<Vec3b>(i, j)[0] = 0;
				erode_img.at<Vec3b>(i, j)[1] = 0;
				erode_img.at<Vec3b>(i, j)[2] = 0;
				pr[i][j] = 0;
				p[i][j] = 0;
				check++;
			}
		}
	}

	if(check==h*w)
	{
		null.x.push_back(0);
		null.y.push_back(0);
		return null;
	}

	null.x.clear();
	null.x.clear();

	Mat erode_img_save = erode_img.clone();

	PathList List=pointFinding(erode_img,p,pr);

	PathList s_list = pathSorted(List, p, erode_img_save);

	return s_list;
}

WallSolution wallFinding(Mat erode_img,int Y_, int X_)
{

	const int h = erode_img.rows;
	const int w = erode_img.cols;
	

	// imshow("image",erode_img);
	// waitKey();

	int** pr_edge = new int* [h];
	for (int i = 0; i < h; i++)
		pr_edge[i] = new int[w];
	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{
			pr_edge[i][j] = erode_img.at<Vec3b>(i, j)[0];

			if (pr_edge[i][j] <= 260 && pr_edge[i][j]>220)
			{
				pr_edge[i][j] = 254;
			}
			else
				pr_edge[i][j] = 0;
		}
	}


	dijkstra02(pr_edge,Y_,X_,h,w);


	int check=0;

	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{
			if (pr_edge[i][j] >= 1000)
			{
				erode_img.at<Vec3b>(i, j)[0] = 254;
				erode_img.at<Vec3b>(i, j)[1] = 254;
				erode_img.at<Vec3b>(i, j)[2] = 254;
				pr_edge[i][j] = 240;
			}
			else
			{
				erode_img.at<Vec3b>(i, j)[0] = 0;
				erode_img.at<Vec3b>(i, j)[1] = 0;
				erode_img.at<Vec3b>(i, j)[2] = 0;
				pr_edge[i][j] = 0;
				check++;
			}
		}
	}

	
	
	if(check==h*w)
	{
		wall_check=1;
		ROS_ERROR("Wall Terminated");
	}
		
	// imshow("image",erode_img);
	// waitKey();
	

	unsigned char** pp = new unsigned char* [h]; 
	for (int i = 0; i < h; i++)
		pp[i] = new unsigned char[w];
	for (int i = 0; i < h; i++)
	{
		for (int j = 0; j < w; j++)
		{
			pp[i][j] = erode_img.at<Vec3b>(i, j)[0];
		}
	}

	vector<int>tempx;
	vector<int>tempy;
		
	WallPath pathFinding_ = dijAlong(pr_edge, h, w, X_, Y_);

		

	WallPath pathFinding;

	pathFinding.Path=new int** [preset_num];
	for (int i = 0; i < preset_num; i++)
		pathFinding.Path[i] = new int* [2];

	pathFinding.Num=0;
	int sub=0;
	for (int i = 0; i < pathFinding_.Num; i++) {
		if (pathFinding_.NumOfPoint[i]>N_point){ 
			pathFinding.Path[i-sub]=pathFinding_.Path[i];
				
			pathFinding.Num = pathFinding.Num + 1;
			pathFinding.NumOfPoint.push_back(pathFinding_.NumOfPoint[i]);
		}
		else
			sub=sub+1;
	}
	

	delete[]pr_edge;

	for (int i=0;i<pathFinding.Num;i++)
	{
		tempx.push_back(pathFinding.Path[i][0][0]);
		tempy.push_back(pathFinding.Path[i][1][0]);
		tempx.push_back(pathFinding.Path[i][0][pathFinding.NumOfPoint[i]-2]);
		tempy.push_back(pathFinding.Path[i][1][pathFinding.NumOfPoint[i]-2]);
	}

	

	PathList temp;
	temp.x=tempx;
	temp.y=tempy;
	WallSolution result;
	result.wall_list=temp;
	result.wall_path=pathFinding;	
	result.p = pp;

	return result;
}
