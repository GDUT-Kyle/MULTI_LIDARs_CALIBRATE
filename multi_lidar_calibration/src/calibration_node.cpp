#include "multi_lidar_calibration/loadTrajAndMap.h"
#include "multi_lidar_calibration/loadBag.h"
#include "multi_lidar_calibration/calibration.h"

int main(int argc, char **argv)
{
    // 实例化ros节点
    ros::init(argc, argv, "calibration");
    ros::NodeHandle nh;

    nh.param<string>("map_path", map_path, "");
    nh.param<string>("traj_path", traj_path, "");
    nh.param<string>("bag_path", bag_path, "");
    // ndt parameter
    nh.param<float>("voxelLeafSize", ndt_config.voxelLeafSize, 0.1);
    nh.param<float>("resolution", ndt_config.resolution, 1.0);
    nh.param<int>("numThreads", ndt_config.numThreads, 8);
    // load initial guass
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


    // 遍历所有livox点云，逐个与map进行ndt匹配
    size_t leftOdomIndex = 0;
    size_t rightOdomIndex = 0;
    Eigen::Matrix<double, 6, 1> alignExtMatSum = Eigen::Matrix<double, 6, 1>::Zero();
    int validCounter = 0;
    auto& Trajs = _map_and_traj_loader.traj;
    auto& Livoxs = bag_loader_.LivoxPcls;
    for(int i=0; i<bag_loader_.LivoxPcls.size()&&ros::ok(); i++)
    {
        // 在导入包的时候就已经确保所有livox消息的时间被包含在odometry的时间跨度内，所以不用做溢出判断
        while(Trajs[rightOdomIndex].timestamp<Livoxs[i].timestamp)
            rightOdomIndex++;
        leftOdomIndex = rightOdomIndex-1;

        double s = (Livoxs[i].timestamp-Trajs[leftOdomIndex].timestamp)/(Trajs[rightOdomIndex].timestamp-Trajs[leftOdomIndex].timestamp);
        
        Transform livoxTrans;
        livoxTrans.timestamp = Livoxs[i].timestamp;
        livoxTrans.rotation = Trajs[rightOdomIndex].rotation.slerp(s, Trajs[leftOdomIndex].rotation);
        livoxTrans.transition = (1.0-s)*Trajs[leftOdomIndex].transition + s*Trajs[rightOdomIndex].transition;

        Eigen::Matrix<double, 4, 4> baseMapMat = Eigen::Matrix<double, 4, 4>::Identity();
        baseMapMat.block(0, 0, 3, 3) = livoxTrans.rotation.toRotationMatrix();
        baseMapMat.block(0, 3, 3, 1) = livoxTrans.transition;
        Eigen::Matrix<double, 4, 4> initGuass = baseMapMat*ExtMatrix; // 配置初始估计

        PointCloud::Ptr outputCloudPtr(new PointCloud);
        double fitnessScore = -1.0;

        #ifdef USE_GICP
        _gicp.setInputSource(Livoxs[i].pointcloud.makeShared());
        _gicp.align(*outputCloudPtr, initGuass.cast<float>());
        Eigen::Matrix4f alignTrans = _gicp.getFinalTransformation();
        fitnessScore = _gicp.getFitnessScore();
        ExtMatrix = (baseMapMat.cast<float>().inverse() *alignTrans).cast<double>();
        #else
        _ndt.setInputSource(Livoxs[i].pointcloud.makeShared());
        _ndt.align(*outputCloudPtr, initGuass.cast<float>());
        Eigen::Matrix4f alignTrans = _ndt.getFinalTransformation();
        // update the extrinsic Matrix (based rslidar)
        fitnessScore = _ndt.getFitnessScore();
        ExtMatrix = (baseMapMat.cast<float>().inverse() *alignTrans).cast<double>();
        #endif

        Eigen::Matrix3d rotaMat = ExtMatrix.block(0, 0, 3, 3);
        Eigen::AngleAxisd angle = Eigen::AngleAxisd(rotaMat);

        // cout<<angle.angle()*(angle.axis().transpose())<<endl<<endl;
        if(fitnessScore < 0.3)
        {
            validCounter++;
            alignExtMatSum.block(0, 0, 3, 1) += angle.angle()*(angle.axis());
            alignExtMatSum.block(3, 0, 3, 1) += ExtMatrix.block(0, 3, 3, 1);
            cout<<alignExtMatSum.transpose()/validCounter<<endl<<endl;
        }

        viewer_final->updatePointCloud<Point>(outputCloudPtr, match_color, "match cloud");
        viewer_final->spinOnce();
    }

    // cout<<alignExtMatSum.transpose()/validCounter<<endl;

    alignExtMatSum = alignExtMatSum/validCounter;
    Eigen::Vector3d angleVec = alignExtMatSum.block(0, 0, 3, 1);
    Eigen::AngleAxisd newExtRotation(angleVec.norm(), angleVec.normalized());
    Eigen::Vector3d newExtTranslate = alignExtMatSum.block(3, 0, 3, 1);
    ExtMatrix.block(0, 0, 3, 3) = newExtRotation.toRotationMatrix();
    ExtMatrix.block(0, 3, 3, 1) = newExtTranslate;
    cout<<"Final result: \n"<<ExtMatrix<<endl;

    viewer_final->close();

    return 0;
}