#include "multi_lidar_calibration/loadTrajAndMap.h"
#include "multi_lidar_calibration/loadBag.h"
#include "multi_lidar_calibration/calibration.h"

// 默认livox的测量频率为10Hz
void distortion(PointCloud::Ptr input, const Transform& startTrans, const Transform& endTrans)
{
    Transform relativeTrans;
    relativeTrans.rotation = startTrans.rotation.inverse()*endTrans.rotation;
    size_t inputSize = input->size();
    // 仅对旋转进行去失真操作
    for(int i=0; i<inputSize; i++)
    {
        float s = 10.0*(input->points[i].intensity-(int)input->points[i].intensity);
        // cout<<s<<", ";
        Eigen::Quaterniond thisRot = Eigen::Quaterniond::Identity().slerp(s, relativeTrans.rotation);
        Eigen::Vector3f thisPoint = Eigen::Vector3f(input->points[i].x, input->points[i].y, input->points[i].z);
        thisPoint = thisRot.cast<float>() * thisPoint;
        input->points[i].x = thisPoint.x();
        input->points[i].y = thisPoint.y();
        input->points[i].z = thisPoint.z();
    }
}

void calibrateLivoxAndMap(bag_loader& LivoxLoader, const loaderTrajAndMap& MapTrajsLoader, 
                const boost::shared_ptr<pcl::visualization::PCLVisualizer> visual,
                const pcl::visualization::PointCloudColorHandlerCustom<Point>& matchColor, 
                Eigen::Matrix4d& initExtMat)
{
    // 遍历所有livox点云，逐个与map进行ndt匹配
    size_t leftOdomIndex = 0;
    size_t rightOdomIndex = 0;
    Eigen::Matrix<double, 6, 1> alignExtMatSum = Eigen::Matrix<double, 6, 1>::Zero();
    int validCounter = 0;
    auto& Trajs = MapTrajsLoader.traj;
    auto& Livoxs = LivoxLoader.LivoxPcls;
    for(int i=0; i<LivoxLoader.LivoxPcls.size()&&ros::ok(); i++)
    {
        // 在导入包的时候就已经确保所有livox消息的时间被包含在odometry的时间跨度内，所以不用做溢出判断
        while(Trajs[rightOdomIndex].timestamp<Livoxs[i].timestamp)
            rightOdomIndex++;
        leftOdomIndex = rightOdomIndex-1;

        double s = (Livoxs[i].timestamp-Trajs[leftOdomIndex].timestamp)/(Trajs[rightOdomIndex].timestamp-Trajs[leftOdomIndex].timestamp);
        
        Transform startPose;
        startPose.timestamp = Livoxs[i].timestamp;
        startPose.rotation = Trajs[leftOdomIndex].rotation.slerp(s, Trajs[rightOdomIndex].rotation);
        startPose.transition = (1.0-s)*Trajs[leftOdomIndex].transition + s*Trajs[rightOdomIndex].transition;

        Eigen::Matrix<double, 4, 4> baseMapMat = Eigen::Matrix<double, 4, 4>::Identity();
        baseMapMat.block(0, 0, 3, 3) = startPose.rotation.toRotationMatrix();
        baseMapMat.block(0, 3, 3, 1) = startPose.transition;
        Eigen::Matrix<double, 4, 4> initGuass = baseMapMat*initExtMat; // 配置初始估计

        // 估计最后一个激光点的时间戳的车辆位姿
        int nextRightOdomIndex = leftOdomIndex;
        while(Trajs[nextRightOdomIndex].timestamp<Livoxs[i].timestamp+0.1)
            nextRightOdomIndex++;
        int nextLeftOdomIndex = nextRightOdomIndex-1;
        if(nextRightOdomIndex<Trajs.size())
        {
            Transform endPose;
            s = (Livoxs[i].timestamp+0.1-Trajs[nextLeftOdomIndex].timestamp)/(Trajs[nextRightOdomIndex].timestamp-Trajs[nextLeftOdomIndex].timestamp);
            if(s>=0.0 || s<=1.0)
            {
                endPose.timestamp = Livoxs[i].timestamp + 0.1; // 10Hz
                endPose.rotation = Trajs[nextLeftOdomIndex].rotation.slerp(s, Trajs[nextRightOdomIndex].rotation);
                endPose.transition = (1.0-s)*Trajs[nextLeftOdomIndex].transition + s*Trajs[nextRightOdomIndex].transition;
                distortion(Livoxs[i].pointcloud.makeShared(), startPose, endPose);
            }
        }

        PointCloud::Ptr outputCloudPtr(new PointCloud);
        double fitnessScore = -1.0;

        #ifdef USE_GICP
        _gicp.setInputSource(Livoxs[i].pointcloud.makeShared());
        _gicp.align(*outputCloudPtr, initGuass.cast<float>());
        Eigen::Matrix4f alignTrans = _gicp.getFinalTransformation();
        fitnessScore = _gicp.getFitnessScore();
        initExtMat = (baseMapMat.cast<float>().inverse() *alignTrans).cast<double>();
        #else
        _ndt.setInputSource(Livoxs[i].pointcloud.makeShared());
        _ndt.align(*outputCloudPtr, initGuass.cast<float>());
        Eigen::Matrix4f alignTrans = _ndt.getFinalTransformation();
        // update the extrinsic Matrix (based rslidar)
        fitnessScore = _ndt.getFitnessScore();
        initExtMat = (baseMapMat.cast<float>().inverse() *alignTrans).cast<double>();
        #endif

        Eigen::Matrix3d rotaMat = initExtMat.block(0, 0, 3, 3);
        Eigen::AngleAxisd angle = Eigen::AngleAxisd(rotaMat);

        if(fitnessScore < 0.3)
        {
            validCounter++;
            alignExtMatSum.block(0, 0, 3, 1) += angle.angle()*(angle.axis());
            alignExtMatSum.block(3, 0, 3, 1) += initExtMat.block(0, 3, 3, 1);
            // cout<<alignExtMatSum.transpose()/validCounter<<endl<<endl;
        }

        visual->updatePointCloud<Point>(outputCloudPtr, matchColor, "match cloud");
        visual->spinOnce();
    }

    alignExtMatSum = alignExtMatSum/validCounter;
    Eigen::Vector3d angleVec = alignExtMatSum.block(0, 0, 3, 1);
    Eigen::AngleAxisd newExtRotation(angleVec.norm(), angleVec.normalized());
    Eigen::Vector3d newExtTranslate = alignExtMatSum.block(3, 0, 3, 1);
    initExtMat.block(0, 0, 3, 3) = newExtRotation.toRotationMatrix();
    initExtMat.block(0, 3, 3, 1) = newExtTranslate;
    cout<<"Final result: \n"<<initExtMat<<endl;
}

int main(int argc, char **argv)
{
    // 实例化ros节点
    ros::init(argc, argv, "calibration");
    ros::NodeHandle nh;

    // Load the map and trajectory generated by A-LOAM
    nh.param<string>("map_path", map_path, "");
    nh.param<string>("traj_path", traj_path, "");
    nh.param<string>("bag_path", bag_path, "");
    // ndt parameter
    nh.param<float>("voxelLeafSize", ndt_config.voxelLeafSize, 0.1);
    nh.param<float>("resolution", ndt_config.resolution, 1.0);
    nh.param<int>("numThreads", ndt_config.numThreads, 8);
    // Load initial guass
    vector<double> extrinsicTrans;
    vector<double> extrinsicRot;
    nh.param<vector<double>>("calibration/extrinsicTrans",  extrinsicTrans, vector<double>());
    nh.param<vector<double>>("calibration/extrinsicRot",  extrinsicRot, vector<double>());
    ExtTrans = Eigen::Map<Eigen::Vector3d>(extrinsicTrans.data());
    ExtRot = Eigen::Map<Eigen::Matrix3d>(extrinsicRot.data());
    ExtMatrix.block(0, 3, 3, 1) = ExtTrans;
    ExtMatrix.block(0, 0, 3, 3) = ExtRot;

    loaderTrajAndMap _map_and_traj_loader;
    _map_and_traj_loader.loadMap(map_path);
    _map_and_traj_loader.loadTraj(traj_path);
    double max_time = _map_and_traj_loader.traj.back().timestamp;
    double min_time = _map_and_traj_loader.traj.front().timestamp;

    bag_loader bag_loader_;
    bag_loader_.loadLivoxsFromROSBag(bag_path, "/livox/lidar", min_time, max_time);

    #ifdef USE_GICP
    if(_map_and_traj_loader.map.size()!=0)
        _gicp.setInputTarget(_map_and_traj_loader.map.makeShared());
    ROS_INFO("The map has been loaded into GICP register...");
    #else
    // ndt omp
    _voxelGridFilter.setLeafSize(ndt_config.voxelLeafSize, ndt_config.voxelLeafSize, ndt_config.voxelLeafSize);
    _ndt.setNumThreads(ndt_config.numThreads);
    _ndt.setTransformationEpsilon(ndt_config.transformationEpsilon);
    _ndt.setStepSize(ndt_config.stepSize);
    _ndt.setResolution(ndt_config.resolution);
    _ndt.setMaximumIterations(ndt_config.maximumIterations);
    _ndt.setNeighborhoodSearchMethod(pclomp::DIRECT7);
    if(_map_and_traj_loader.map.size()!=0)
        _ndt.setInputTarget(_map_and_traj_loader.map.makeShared());
    ROS_INFO("The map has been loaded into NDT register...");
    #endif

    //prepare display
    boost::shared_ptr<pcl::visualization::PCLVisualizer>
        viewer_final(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<Point> map_color(_map_and_traj_loader.map.makeShared(), 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<Point> match_color(bag_loader_.LivoxPcls[0].pointcloud.makeShared(), 0, 255, 0);

    viewer_final->addPointCloud<Point>(_map_and_traj_loader.map.makeShared(), map_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
    viewer_final->addPointCloud<Point>(bag_loader_.LivoxPcls[0].pointcloud.makeShared(), match_color, "match cloud"); //display the match cloud

    calibrateLivoxAndMap(bag_loader_, _map_and_traj_loader, viewer_final, match_color, ExtMatrix);
    // Other Livox LiDAR
    // calibrateLivoxAndMap(bag_loader_, _map_and_traj_loader, viewer_final, match_color, ExtMatrix);
    
    viewer_final->close();

    return 0;
}