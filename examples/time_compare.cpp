/*
    Description: An example for using ikd-Tree
    Author: Yixi Cai
    Email: yixicai@connect.hku.hk
*/

#include <ikd_Tree.h>
#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <algorithm>
#include "pcl/point_types.h"
#include "pcl/common/common.h"
#include "pcl/point_cloud.h"
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/octree/octree_search.h>  //octree相关定义
#include "Octree.h"
#include <pcl/io/ply_io.h>
#include<sstream>
#if defined _MSC_VER
#include <direct.h>
#elif defined __GNUC__
#include <sys/types.h>
#include <sys/stat.h>
#endif

// using PointType = ikdTree_PointType;
using PointType = pcl::PointXYZ;
using PointVector = KD_TREE<PointType>::PointVector;


#define X_MAX 5.0
#define X_MIN -5.0
#define Y_MAX 5.0
#define Y_MIN -5.0
#define Z_MAX 5.0
#define Z_MIN -5.0

#define Point_Num 400000
#define New_Point_Num 2000
#define Delete_Point_Num 100
#define Nearest_Num 5
#define Radius 0.3
#define Test_Time 100
#define Search_Counter 200
#define Radius_Search_Counter 200
#define Box_Length 1.5
#define Box_Num 4
#define Delete_Box_Switch true
#define Add_Box_Switch true

// PointVector point_cloud;
pcl::PointCloud<PointType> point_cloud, cloud_increment, cloud_decrement, cloud_deleted, search_result, raw_cmp_result, DeletePoints, removed_points;


KD_TREE<PointType> ikd_Tree(0.3,0.6,0.2);


float rand_float(float x_min, float x_max){
    float rand_ratio = rand()/(float)RAND_MAX;
    return (x_min + rand_ratio * (x_max - x_min));
}

// Generate the points to initialize an incremental k-d tree
void generate_initial_point_cloud(int num){
    PointVector ().swap(point_cloud.points);
    PointType new_point;
    for (int i=0;i<num;i++){
        new_point.x = rand_float(X_MIN, X_MAX);
        new_point.y = rand_float(Y_MIN, Y_MAX);
        new_point.z = rand_float(Z_MIN, Z_MAX);
        point_cloud.push_back(new_point);
    }
    return;
}

// Generate random new points for point-wise insertion to the incremental k-d tree
void generate_increment_point_cloud(int num){
    PointVector ().swap(cloud_increment.points);
    PointType new_point;
    for (int i=0;i<num;i++){
        new_point.x = rand_float(X_MIN, X_MAX);
        new_point.y = rand_float(Y_MIN, Y_MAX);
        new_point.z = rand_float(Z_MIN, Z_MAX);
        point_cloud.push_back(new_point);
        cloud_increment.push_back(new_point);        
    }
    return;
}

// Generate random points for point-wise delete on the incremental k-d tree
void generate_decrement_point_cloud(int num){
    PointVector ().swap(cloud_decrement.points);
    auto rng = default_random_engine();
    shuffle(point_cloud.points.begin(), point_cloud.points.end(), rng);    
    for (int i=0;i<num;i++){
        cloud_decrement.push_back(point_cloud[point_cloud.size()-1]);
        point_cloud.points.pop_back();
    }
    return;
}

// Generate random boxes for box-wise re-insertion on the incremental k-d tree
void generate_box_increment(vector<BoxPointType> & Add_Boxes, float box_length, int box_num){
    vector<BoxPointType> ().swap(Add_Boxes);
    float d = box_length/2;
    float x_p, y_p, z_p;
    BoxPointType boxpoint;
    for (int k=0;k < box_num; k++){
        x_p = rand_float(X_MIN, X_MAX);
        y_p = rand_float(Y_MIN, Y_MAX);
        z_p = rand_float(Z_MIN, Z_MAX);        
        boxpoint.vertex_min[0] = x_p - d;
        boxpoint.vertex_max[0] = x_p + d;
        boxpoint.vertex_min[1] = y_p - d;
        boxpoint.vertex_max[1] = y_p + d;  
        boxpoint.vertex_min[2] = z_p - d;
        boxpoint.vertex_max[2] = z_p + d;
        Add_Boxes.push_back(boxpoint);
        int n = cloud_deleted.size();
        int counter = 0;
        while (counter < n){
            PointType tmp = cloud_deleted[cloud_deleted.size()-1];
            cloud_deleted.points.pop_back();
        
            if (tmp.x +EPSS < boxpoint.vertex_min[0] || tmp.x - EPSS > boxpoint.vertex_max[0] || tmp.y + EPSS < boxpoint.vertex_min[1] || tmp.y - EPSS > boxpoint.vertex_max[1] || tmp.z + EPSS < boxpoint.vertex_min[2] || tmp.z - EPSS > boxpoint.vertex_max[2]){
                cloud_deleted.points.insert(cloud_deleted.begin(),tmp);
            } else {            
                point_cloud.push_back(tmp);
            }
            counter += 1;
        }
    }
}

// Generate random boxes for box-wise delete on the incremental k-d tree
void generate_box_decrement(vector<BoxPointType> & Delete_Boxes, float box_length = 1.0, int box_num=1, bool return_clear_data = false)
{
    vector<BoxPointType> ().swap(Delete_Boxes);
    float d = box_length/2;
    float x_p, y_p, z_p;
    BoxPointType boxpoint;
    for (int k=0;k < box_num; k++){
        x_p = rand_float(X_MIN, X_MAX);
        y_p = rand_float(Y_MIN, Y_MAX);
        z_p = rand_float(Z_MIN, Z_MAX);        
        boxpoint.vertex_min[0] = x_p - d;
        boxpoint.vertex_max[0] = x_p + d;
        boxpoint.vertex_min[1] = y_p - d;
        boxpoint.vertex_max[1] = y_p + d;  
        boxpoint.vertex_min[2] = z_p - d;
        boxpoint.vertex_max[2] = z_p + d;
        Delete_Boxes.push_back(boxpoint);
        int n = point_cloud.size();
        // if(!return_clear_data) continue;
        int counter = 0;
        pcl::PointCloud<PointType> point_cloud_tmp;
        point_cloud_tmp.points.resize(n);
        for(int i=0;i<n;i++)
        {
            PointType tmp = point_cloud[i];
            if (tmp.x + EPSS < boxpoint.vertex_min[0] || tmp.x - EPSS > boxpoint.vertex_max[0] || 
                tmp.y + EPSS < boxpoint.vertex_min[1] || tmp.y - EPSS > boxpoint.vertex_max[1] || 
                tmp.z + EPSS < boxpoint.vertex_min[2] || tmp.z - EPSS > boxpoint.vertex_max[2])
            {
                point_cloud_tmp[counter] = tmp;
                counter++;
            } 
            else 
            {
                cloud_deleted.points.push_back(tmp);
            }
        }
        point_cloud_tmp.points.resize(counter);
        point_cloud.points.swap(point_cloud_tmp.points);
    }
}

// Generate target point for nearest search on the incremental k-d tree
PointType generate_target_point(){
    PointType point;
    point.x = rand_float(X_MIN, X_MAX);;
    point.y = rand_float(Y_MIN, Y_MAX);
    point.z = rand_float(Z_MIN, Z_MAX);
    return point;
}

void test_read_txt(const std::string & filename)
{
	std::ifstream infile(filename.c_str());
    if(!infile.is_open())
    {
        std::cout<<"Can not open file: "<<filename<<std::endl;
        return;
    }
    uint32_t ordered_indies[8][8][63][2];
    int first_oct_id=0, second_oct_id=0, indies_for_one_id = 0;
	while (!infile.eof())
	{
		std::string s;
		std::getline(infile, s);
		if (!s.empty())
		{
            // std::cout<<s<<std::endl;
			std::stringstream ss;
			ss << s;
            ss>>ordered_indies[first_oct_id][second_oct_id][indies_for_one_id][0];
            ss>>ordered_indies[first_oct_id][second_oct_id][indies_for_one_id][1];
            indies_for_one_id++;
            if(indies_for_one_id>62)
            {
                indies_for_one_id=0;
                second_oct_id++;
                if(second_oct_id>7)
                {
                    second_oct_id = 0;
                    first_oct_id++;
                    if(first_oct_id>7)
                    {
                        break;
                    }
                }
            }
		}
	}
    for(int i=0;i<8;i++)
        for(int j=0;j<8;j++)
        {
            std::cout<<i<<", "<<j<<": ";
            for(int k=0;k<63;k++)
            {
                std::cout<<"["<<ordered_indies[i][j][k][0]<<", "<<ordered_indies[i][j][k][1]<<"] ";
            }
            std::cout<<std::endl<<std::endl;
        }
	infile.close();
}

std::string Convert(float Num, int fixed=3)
{
    std::ostringstream oss;
    oss<<Num;
    std::string str(oss.str());
    return str;
}

inline void create_dir(std::string dir)
{
    #if defined _MSC_VER
        _mkdir(dir.data());
    #elif defined __GNUC__
        mkdir(dir.data(), 0777);
    #endif
}

inline bool if_file_exist( const std::string &name )
{
    //Copy from: https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exist-using-standard-c-c11-c
    struct stat buffer;
    return ( stat( name.c_str(), &buffer ) == 0 );
}

void test_octree_incre()
{
    generate_initial_point_cloud(Point_Num);
    pcl::PointCloud<PointType>::Ptr point_cloud_octree(new pcl::PointCloud<PointType>());
    *point_cloud_octree += point_cloud;
    pcl::octree::OctreePointCloudSearch<PointType> octree(0.01f); //建立octree对象
    octree.setInputCloud(point_cloud_octree); //传入需要建立kdtree的点云指针
    octree.addPointsFromInputCloud();
    generate_increment_point_cloud(New_Point_Num);
    for(int i=0;i<cloud_increment.size();i++)
    {
        octree.addPointToCloud(cloud_increment[i], point_cloud_octree);
    }
    std::cout<<"test_octree_incre"<<std::endl;
}

// ===随机数据集算法对比实验===
void random_dataset_comparison()
{
    srand((unsigned) time(NULL));
    printf("Testing ...\n");
	std::string m_map_output_dir= "/media/zhujun/0DFD06D20DFD06D2/SLAM/octree_test/ikd-Tree-main-github/examples/output";
    m_map_output_dir = m_map_output_dir + "/random_dataset";
    if(!if_file_exist(m_map_output_dir))
    {
        create_dir(m_map_output_dir);
    }
    {
        std::ofstream log_file(m_map_output_dir + "/incremental_updates_time.txt", std::ios::out);
        log_file<<"# n i_octree ikd pcl_octree"<<std::endl;
        log_file.close();
    }
    {
        std::ofstream log_file(m_map_output_dir + "/knn_time.txt", std::ios::out);
        log_file<<"# n i_octree ikd pcl_octree"<<std::endl;
        log_file.close();
    }
    {
        std::ofstream log_file(m_map_output_dir + "/radius_time.txt", std::ios::out);
        log_file<<"# n  i_octree ikd pcl_octree"<<std::endl;
        log_file.close();
    }
    thuni::Octree i_octree2;
    pcl::octree::OctreePointCloudSearch<PointType> octree(0.01f); //建立octree对象
    pcl::PointCloud<PointType>::Ptr point_cloud_octree(new pcl::PointCloud<PointType>());
    float i_oct_time, i_oct2_time, pcl_kd_time, ikd_time, pcl_octree_time, libnabo_time, ann_time, flann_time;
    int counter = 0;
    PointType target; 
    bool test_result = false; // false  true
    auto begin = chrono::high_resolution_clock::now();
    auto end = chrono::high_resolution_clock::now();    
    auto build_duration = chrono::duration_cast<chrono::microseconds>(end-begin).count();
    // ==============初始化=====================
    generate_initial_point_cloud(Point_Num);
    {
        begin = chrono::high_resolution_clock::now();
		i_octree2.initialize(point_cloud); // 0.006661
		end = chrono::high_resolution_clock::now();
		i_oct2_time = float(chrono::duration_cast<chrono::microseconds>(end-begin).count())/1e3;

		// i-kdtree
        begin = chrono::high_resolution_clock::now();
        ikd_Tree.Build(point_cloud.points); 
		end = chrono::high_resolution_clock::now();
		ikd_time = float(chrono::duration_cast<chrono::microseconds>(end-begin).count())/1e3;

		// pcl octree
        *point_cloud_octree += point_cloud;
        begin = chrono::high_resolution_clock::now();
        octree.setInputCloud(point_cloud_octree); //传入需要建立kdtree的点云指针
        octree.addPointsFromInputCloud();  //构建Octree
		end = chrono::high_resolution_clock::now();
        pcl_octree_time = float(chrono::duration_cast<chrono::microseconds>(end-begin).count())/1e3;

        std::ofstream log_file(m_map_output_dir + "/incremental_updates_time.txt", std::ios::app);
		log_file    <<std::fixed << std::setprecision(7)
					<<0<<" " // lidar_beg_time
                    <<i_oct2_time<<" "
					<<ikd_time<<" "
                    <<pcl_octree_time<<" "
					<<std::endl;
		log_file.close();
	}
    while (counter < Test_Time)
    {
        printf("Test %d/%d\n",counter+1, Test_Time);
        // =======================（增量式）创建===========================
        generate_increment_point_cloud(New_Point_Num);
        {
            // i-Octree2
            begin = chrono::high_resolution_clock::now();
            i_octree2.update(cloud_increment); // 0.006661
            end = chrono::high_resolution_clock::now();
            i_oct2_time = float(chrono::duration_cast<chrono::microseconds>(end-begin).count())/1e3;

            // i-kdtree
            begin = chrono::high_resolution_clock::now();
            ikd_Tree.Add_Points(cloud_increment.points, false);
            end = chrono::high_resolution_clock::now();
            ikd_time = float(chrono::duration_cast<chrono::microseconds>(end-begin).count())/1e3;

            // pcl octree
            begin = chrono::high_resolution_clock::now();
            {
                for(int i=0;i<cloud_increment.size();i++)
                {
                    octree.addPointToCloud(cloud_increment[i], point_cloud_octree);
                }
            }
            end = chrono::high_resolution_clock::now();
            pcl_octree_time = float(chrono::duration_cast<chrono::microseconds>(end-begin).count())/1e3;

            std::ofstream log_file(m_map_output_dir + "/incremental_updates_time.txt", std::ios::app);
            log_file    <<std::fixed << std::setprecision(7)
                        <<counter+1<<" " // lidar_beg_time
                        <<i_oct2_time<<" "
                        <<ikd_time<<" "
                        <<pcl_octree_time<<" "
                        <<std::endl;
            log_file.close();
        }

        // =============================Nearest Search =================================
        // if(0)
        {
            auto search_duration = chrono::duration_cast<chrono::microseconds>(end-end).count();
            i_oct_time = i_oct2_time = pcl_kd_time = ikd_time = pcl_octree_time = libnabo_time = ann_time = flann_time = 0.0f;
            float error_flag=0;
            for (int k=0;k<Search_Counter;k++)
            {
                target = generate_target_point();
                // i-Octree2
                {
                    std::vector<size_t> results;
                    std::vector<float> dist_results;
                    begin = chrono::high_resolution_clock::now();
                    i_octree2.knnNeighbors(target, Nearest_Num, results, dist_results);
                    end = chrono::high_resolution_clock::now();
                    i_oct2_time += float(chrono::duration_cast<chrono::microseconds>(end-begin).count())/1e3;
                    if(test_result && k==0)
                    {
                        std::cout<<"i_octree2:"<<std::endl;
                        for(int i=0;i<Nearest_Num;i++)
                        {
                            std::cout<<sqrt(dist_results[i])<<" ";
                        }
                        std::cout<<std::endl;
                    }
                }
                // i-kdtree
                {
                    PointVector ().swap(search_result.points);
                    std::vector<float> pointNKNSquaredDistance;
                    begin = chrono::high_resolution_clock::now();
                    ikd_Tree.Nearest_Search(target, Nearest_Num, search_result.points, pointNKNSquaredDistance);
                    end = chrono::high_resolution_clock::now();
                    ikd_time += float(chrono::duration_cast<chrono::microseconds>(end-begin).count())/1e3;
                    if(test_result && k==0)
                    {
                        std::cout<<"ikd_Tree: "<<std::endl;
                        for(int i=0;i<Nearest_Num;i++)
                        {
                            std::cout<<sqrt(pointNKNSquaredDistance[i])<<" ";
                        }
                        std::cout<<std::endl;
                    }
                }
                // pcl octree
                {
                    std::vector<int> pointIdxNKNSearch;
                    std::vector<float> pointNKNSquaredDistance;
                    begin = chrono::high_resolution_clock::now();
                    octree.nearestKSearch(target, Nearest_Num, pointIdxNKNSearch,pointNKNSquaredDistance);
                    end = chrono::high_resolution_clock::now();
                    pcl_octree_time += float(chrono::duration_cast<chrono::microseconds>(end-begin).count())/1e3;
                    if(test_result && k==0)
                    {
                        std::cout<<"octree:\n";
                        for(int i=0;i<Nearest_Num;i++)
                        {
                            std::cout<<sqrt(pointNKNSquaredDistance[i])<<" ";
                        }
                        std::cout<<std::endl;
                    }
                }

            }
            // knn 时间记录
            {
                std::ofstream log_file(m_map_output_dir + "/knn_time.txt", std::ios::app);
                log_file    <<std::fixed << std::setprecision(7)
                            <<counter+1<<" " // lidar_beg_time
                            <<i_oct2_time<<" "
                            <<ikd_time<<" "
                            <<pcl_octree_time<<" "
                            <<std::endl;
                log_file.close();
            }
        }
        // =============================Radius Search ===================================
        if(1)
        {
            auto search_duration = chrono::duration_cast<chrono::microseconds>(end-end).count();
            i_oct_time = i_oct2_time = pcl_kd_time = ikd_time = pcl_octree_time = libnabo_time = ann_time = flann_time = 0.0f;
            float error_flag=0;
            for (int k=0;k<Radius_Search_Counter;k++)
            {
                target = generate_target_point();

                // i-Octree2
                {
                    std::vector<size_t> results;
                    std::vector<float> dist_results;
                    begin = chrono::high_resolution_clock::now();
                    i_octree2.radiusNeighbors(target, float(Radius), results, dist_results);
                    end = chrono::high_resolution_clock::now();
                    i_oct2_time += float(chrono::duration_cast<chrono::microseconds>(end-begin).count())/1e3;
                    if(test_result && k==0)
                    {
                        std::sort(dist_results.begin(), dist_results.end());
                        std::cout<<"i_octree2:"<<std::endl;
                        for(int i=0;i<dist_results.size();i++)
                        {
                            std::cout<<sqrt(dist_results[i])<<" ";
                        }
                        std::cout<<std::endl;
                    }
                }

                // i-kdtree
                {
                    PointVector ().swap(search_result.points);
                    std::vector<float> pointNKNSquaredDistance;
                    begin = chrono::high_resolution_clock::now();
                    ikd_Tree.Radius_Search(target, float(Radius), search_result.points);
                    end = chrono::high_resolution_clock::now();
                    ikd_time += float(chrono::duration_cast<chrono::microseconds>(end-begin).count())/1e3;
                    if(test_result && k==0)
                    {
                        pointNKNSquaredDistance.resize(search_result.points.size(),-1.0f);
                        for(size_t i =0; i<search_result.points.size(); i++)
                        {
                            pointNKNSquaredDistance[i] = std::pow(search_result.points[i].x-target.x, 2)+std::pow(search_result.points[i].y-target.y, 2)+std::pow(search_result.points[i].z-target.z, 2);
                        }
                        std::sort(pointNKNSquaredDistance.begin(), pointNKNSquaredDistance.end());
                        std::cout<<"ikd_Tree: "<<std::endl;
                        for(int i=0;i<pointNKNSquaredDistance.size();i++)
                        {
                            std::cout<<sqrt(pointNKNSquaredDistance[i])<<" ";
                        }
                        std::cout<<std::endl;
                    }
                }
                size_t num_radius = 1; // for flann
                // pcl octree
                {
                    std::vector<int> pointIdxNKNSearch;
                    std::vector<float> pointNKNSquaredDistance;
                    begin = chrono::high_resolution_clock::now();
                    octree.radiusSearch(target, Radius, pointIdxNKNSearch,pointNKNSquaredDistance);
                    end = chrono::high_resolution_clock::now();
                    pcl_octree_time += float(chrono::duration_cast<chrono::microseconds>(end-begin).count())/1e3;
                    num_radius = pointNKNSquaredDistance.size();
                    if(test_result && k==0)
                    {
                        std::sort(pointNKNSquaredDistance.begin(), pointNKNSquaredDistance.end());
                        std::cout<<"octree:\n";
                        for(int i=0;i<pointNKNSquaredDistance.size();i++)
                        {
                            std::cout<<sqrt(pointNKNSquaredDistance[i])<<" ";
                        }
                        std::cout<<std::endl;
                    }
                }
            }
            // radius search 时间记录
            {
                std::ofstream log_file(m_map_output_dir + "/radius_time.txt", std::ios::app);
                log_file    <<std::fixed << std::setprecision(7)
                            <<counter+1<<" " // lidar_beg_time
                            <<i_oct2_time<<" "
                            <<ikd_time<<" "
                            <<pcl_octree_time<<" "
                            <<std::endl;
                log_file.close();
            }
        }
        counter += 1;    
    }
    printf("Finished %d times test\n",counter);
}
int main(int argc, char** argv){
    random_dataset_comparison();
    return 1;
}


