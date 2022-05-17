#include "multi_lidar_calibration/common.h"
#include <pcl/visualization/cloud_viewer.h>
#ifdef USE_GICP
#include <pclomp/gicp_omp.h>
#else
#include <pclomp/ndt_omp.h>
#endif

using namespace std;

#ifdef USE_GICP
typedef pclomp::GeneralizedIterativeClosestPoint<Point, Point> GICP;
#else
typedef pclomp::NormalDistributionsTransform<Point, Point> NDT;
#endif

struct NDT_CONFIG
{
    bool debug = false;
    int numThreads = 4;
    int maximumIterations = 20;
    float voxelLeafSize = 0.1;
    float resolution = 1.0;
    double transformationEpsilon = 0.01;
    double stepSize = 0.1;
    double threshShift = 2;
    double threshRot = M_PI / 12;
    double minScanRange = 1.0;
    double maxScanRange = 100;
} ndt_config;

string map_path, traj_path, bag_path;
Eigen::Vector3d ExtTrans;
Eigen::Matrix3d ExtRot;
Eigen::Matrix<double, 4, 4> ExtMatrix;

#ifdef USE_GICP
GICP _gicp;
#else
NDT _ndt;
pcl::VoxelGrid<Point> _voxelGridFilter;
#endif