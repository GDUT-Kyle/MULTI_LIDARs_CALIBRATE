#ifndef _LOAD_TRAJ_AND_MAP_H_
#define _LOAD_TRAJ_AND_MAP_H_

#include "common.h"

using namespace std;

class loaderTrajAndMap
{
public:
    PointCloud map;
    vector<Transform> traj;
public:
    loaderTrajAndMap(){};
    virtual ~loaderTrajAndMap(){};
    bool loadMap(const string& path);
    bool loadTraj(const string& path);
};

bool loaderTrajAndMap::loadMap(const string& path)
{
    ROS_INFO("Loading map ...");

    // 从文件导入指定pcd文件点云地图
    // pcl::io::loadPCDFile<PointType> (map_path, *map_cloud);
    pcl::io::loadPCDFile(path, map);

    std::vector<int> indices; //保存去除的点的索引
    pcl::removeNaNFromPointCloud(map, map, indices); //去除点云中的NaN点

    // pcl::transformPointCloud(*map_cloud, *map_cloud, map2cam);

    // cout<<"map_cloud->size() = "<<map_cloud->size()<<endl;
    ROS_INFO("map_cloud->size() = %lu", map.size());
    // 检测文件是否有效
    if (map.size()==0){
        ROS_INFO("The map is empty...");
        return false;
    }

    return true;
}

bool loaderTrajAndMap::loadTraj(const string& path)
{
    ifstream inFile(path, ios::in);
    if (!inFile)
    {
        cout << "Failed to open file!" << endl;
        exit(1);
    }
    string line;
    string field;
    while(getline(inFile, line))
    {
        istringstream sin(line);
        double trans[8];
        int i = 0;
        while(getline(sin, field, ' '))
        {
            trans[i] = atof(field.c_str());
            i++;
        }
        traj.push_back(Transform(trans[0], Eigen::Vector3d(trans[1], trans[2], trans[3]), 
                        Eigen::Quaterniond(trans[7], trans[4], trans[5], trans[6])));
    }
    // 检测文件是否有效
    if (traj.size()==0){
        ROS_INFO("The trajectory is empty...");
        return false;
    }
    inFile.close();

    ROS_INFO("The size of trajectory = %lu", traj.size());

    return true;
}


#endif