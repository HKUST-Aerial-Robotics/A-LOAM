#include <cmath>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv/cv.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>

#include "loam_velodyne/common.h"
#include "lidarFactor_v6.cpp"

#define BACKWARD_HAS_DW 1	

#define DISTORTION 0

#include <backward.hpp>	
namespace backward	
{	
backward::SignalHandling sh;	
} // namespace backward

int corner_correspondence = 0, plane_correspondence = 0;

constexpr double SCAN_PERIOD = 0.1;
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;

const int skipFrameNum = 1;
bool systemInited = false;

double timeCornerPointsSharp = 0;
double timeCornerPointsLessSharp = 0;
double timeSurfPointsFlat = 0;
double timeSurfPointsLessFlat = 0;
double timeLaserCloudFullRes = 0;

bool newCornerPointsSharp = false;
bool newCornerPointsLessSharp = false;
bool newSurfPointsFlat = false;
bool newSurfPointsLessFlat = false;
bool newLaserCloudFullRes = false;

pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());
pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZI>());

pcl::PointCloud<PointType>::Ptr cornerPointsSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsFlat(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudOri(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr coeffSel(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());

int laserCloudCornerLastNum = 0;
int laserCloudSurfLastNum = 0;

//save 2 corner points index searched
size_t pointSearchCornerInd1[200000];
size_t pointSearchCornerInd2[200000];

//save 3 surf points index searched
size_t pointSearchSurfInd1[200000];
size_t pointSearchSurfInd2[200000];
size_t pointSearchSurfInd3[200000];

// Transformation from current frame to world frame
Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
Eigen::Vector3d t_w_curr(0, 0, 0);

// q_curr_last(x, y, z, w), t_curr_last
double para_q[4] = {0, 0, 0, 1};
double para_t[4] = {0, 0, 0};

Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
Eigen::Map<Eigen::Vector3d>    t_last_curr(para_t);


#define TIME_STILL 1

inline Eigen::Matrix3d RPY2RotationMatrix(double roll, double pitch, double yaw)
{
    return (Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())).toRotationMatrix();
}

void getGTPose(double timestamp, Eigen::Matrix3d &R_w_curr, Eigen::Vector3d &t_w_curr)
{
    double LINE_VELOCITY = 0.2;  // velocity: 0.2 m/s
    // NOTE: some velocity may cause multiple solution to a specific point due to lidar rotation and scan
    // double roll_velocity = 0;
    // double pitch_velocity = 0;
    // double yaw_velocity = 0;
    double ROLL_VELOCITY = 0 * M_PI;
    double PITCH_VELOCITY = 0.02 * M_PI;
    double YAW_VELOCITY = 0.02 * M_PI;

    if (timestamp < TIME_STILL)
    {
        R_w_curr = Eigen::Matrix3d::Identity();
        t_w_curr = Eigen::Vector3d::Zero();
    }
    else
    {
        double move_time = timestamp - TIME_STILL;

        R_w_curr = RPY2RotationMatrix(ROLL_VELOCITY*move_time, PITCH_VELOCITY*move_time, YAW_VELOCITY*move_time);
        t_w_curr << move_time*LINE_VELOCITY, move_time*LINE_VELOCITY, sin(10*move_time*LINE_VELOCITY);
    }

}

void getRelativeGTPose(double timestamp_curr, double timestamp_last, Eigen::Matrix3d &R_last_curr, Eigen::Vector3d &t_last_curr)
{
    Eigen::Matrix3d R_curr, R_last;
    Eigen::Vector3d t_curr, t_last;

    getGTPose(timestamp_curr, R_curr, t_curr);
    getGTPose(timestamp_last, R_last, t_last);

    Eigen::Matrix4d T_curr = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d T_last = Eigen::Matrix4d::Identity();

    T_curr.block<3, 3>(0, 0) = R_curr;
    T_curr.block<3, 1>(0, 3) = t_curr;

    T_last.block<3, 3>(0, 0) = R_last;
    T_last.block<3, 1>(0, 3) = t_last;

    Eigen::Matrix4d T_last_curr = T_last.inverse() * T_curr;

    R_last_curr = T_last_curr.block<3, 3>(0, 0);
    t_last_curr = T_last_curr.block<3, 1>(0, 3);
}

// undistort lidar point
void TransformToStart(PointType const *const pi, PointType *const po)
{
    //interpolation ratio
    double s;
    if (DISTORTION)
        s = (pi->intensity - int(pi->intensity)) / SCAN_PERIOD;
    else
        s = 1.0;
    //s = 1;
    Eigen::Quaterniond q_point_last = Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
    Eigen::Vector3d t_point_last = s * t_last_curr;
    // Eigen::Vector3d t_point_last = s * t_curr_last;
    Eigen::Vector3d point(pi->x, pi->y, pi->z);
    Eigen::Vector3d un_point = q_point_last * point + t_point_last;

    po->x = un_point.x();
    po->y = un_point.y();
    po->z = un_point.z();
    po->intensity = pi->intensity;
}

// transform all lidar points to the start of the next frame

void TransformToEnd(PointType const *const pi, PointType *const po)
{
    // undistort point first
    pcl::PointXYZI un_point_tmp;
    TransformToStart(pi, &un_point_tmp);

    Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
    Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

    po->x = point_end.x();
    po->y = point_end.y();
    po->z = point_end.z();

    //Remove distortion time info
    po->intensity = int(pi->intensity);
}


void laserCloudSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsSharp2)
{
    timeCornerPointsSharp = cornerPointsSharp2->header.stamp.toSec();

    cornerPointsSharp->clear();
    pcl::fromROSMsg(*cornerPointsSharp2, *cornerPointsSharp);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cornerPointsSharp, *cornerPointsSharp, indices);
    newCornerPointsSharp = true;
}

void laserCloudLessSharpHandler(const sensor_msgs::PointCloud2ConstPtr &cornerPointsLessSharp2)
{
    timeCornerPointsLessSharp = cornerPointsLessSharp2->header.stamp.toSec();

    cornerPointsLessSharp->clear();
    pcl::fromROSMsg(*cornerPointsLessSharp2, *cornerPointsLessSharp);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cornerPointsLessSharp, *cornerPointsLessSharp, indices);
    newCornerPointsLessSharp = true;
}

void laserCloudFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsFlat2)
{
    timeSurfPointsFlat = surfPointsFlat2->header.stamp.toSec();

    surfPointsFlat->clear();
    pcl::fromROSMsg(*surfPointsFlat2, *surfPointsFlat);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*surfPointsFlat, *surfPointsFlat, indices);
    newSurfPointsFlat = true;
}

void laserCloudLessFlatHandler(const sensor_msgs::PointCloud2ConstPtr &surfPointsLessFlat2)
{
    timeSurfPointsLessFlat = surfPointsLessFlat2->header.stamp.toSec();

    surfPointsLessFlat->clear();
    pcl::fromROSMsg(*surfPointsLessFlat2, *surfPointsLessFlat);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*surfPointsLessFlat, *surfPointsLessFlat, indices);
    newSurfPointsLessFlat = true;
}

//receive all point cloud
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
    timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();

    laserCloudFullRes->clear();
    pcl::fromROSMsg(*laserCloudFullRes2, *laserCloudFullRes);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*laserCloudFullRes, *laserCloudFullRes, indices);
    newLaserCloudFullRes = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "laserOdometry");
    ros::NodeHandle nh;

    ros::Subscriber subCornerPointsSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_sharp", 2, laserCloudSharpHandler);

    ros::Subscriber subCornerPointsLessSharp = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_sharp", 2, laserCloudLessSharpHandler);

    ros::Subscriber subSurfPointsFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_flat", 2, laserCloudFlatHandler);

    ros::Subscriber subSurfPointsLessFlat = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_less_flat", 2, laserCloudLessFlatHandler);

    ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_2", 2, laserCloudFullResHandler);

    ros::Publisher pubLaserCloudCornerLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2);

    ros::Publisher pubLaserCloudSurfLast = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2);

    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 2);

    ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_to_init", 5);

    ros::Publisher pubCurrToLastPointCloud = nh.advertise<sensor_msgs::PointCloud2>("/curr_to_last_point_cloud", 2);

    ros::Publisher pubCurrToLastPlanePointCloud = nh.advertise<sensor_msgs::PointCloud2>("/curr_to_last_plane_point_cloud", 2);

    ros::Publisher pubLastPointCloud = nh.advertise<sensor_msgs::PointCloud2>("/last_point_cloud", 2);

    ros::Publisher pubLastPlanePointCloud = nh.advertise<sensor_msgs::PointCloud2>("/last_plane_point_cloud", 2);


    nav_msgs::Odometry laserOdometry;
    laserOdometry.header.frame_id = "/camera_init";
    laserOdometry.child_frame_id = "/laser_odom";

    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform laserOdometryTrans;
    laserOdometryTrans.frame_id_ = "/camera_init";
    laserOdometryTrans.child_frame_id_ = "/laser_odom";

    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_path", 5);
    nav_msgs::Path laserPath;
    laserPath.header.frame_id = "/camera_init";

    int frameCount = skipFrameNum;
    ros::Rate rate(100);

    while(ros::ok())
    {
        ros::spinOnce();
        if (newCornerPointsSharp && newCornerPointsLessSharp && newSurfPointsFlat && 
            newSurfPointsLessFlat && newLaserCloudFullRes &&
            fabs(timeCornerPointsSharp - timeSurfPointsLessFlat) < 0.005 &&
            fabs(timeCornerPointsLessSharp - timeSurfPointsLessFlat) < 0.005 &&
            fabs(timeSurfPointsFlat - timeSurfPointsLessFlat) < 0.005 &&
            fabs(timeLaserCloudFullRes - timeSurfPointsLessFlat) < 0.005)
        {
            newCornerPointsSharp = false;
            newCornerPointsLessSharp = false;
            newSurfPointsFlat = false;
            newSurfPointsLessFlat = false;
            newLaserCloudFullRes = false;

            // initializing
            if (!systemInited)
            {
                pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
                cornerPointsLessSharp = laserCloudCornerLast;
                laserCloudCornerLast = laserCloudTemp;

                laserCloudTemp = surfPointsLessFlat;
                surfPointsLessFlat = laserCloudSurfLast;
                laserCloudSurfLast = laserCloudTemp;

                laserCloudCornerLastNum = laserCloudCornerLast->points.size();
                laserCloudSurfLastNum = laserCloudSurfLast->points.size();

                // construct kdtree
                printf("laserCloudCornerLast size %d\n", laserCloudCornerLast->points.size());
                kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
                printf("laserCloudSurfLast size %d\n", laserCloudSurfLast->points.size());
                kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

/*                 std::cout << "****************************************************************\n";
                for (int i = 0; i < laserCloudCornerLast->points.size(); ++i)
                {
                std::cout << laserCloudCornerLast->points[i].x << ", "
                            << laserCloudCornerLast->points[i].y << ", "
                            << laserCloudCornerLast->points[i].z << ", "
                            << laserCloudCornerLast->points[i].intensity << '\n';
                }
                std::cout << "****************************************************************\n"; */

                // send corner and plane features from last frame to mapping node
                sensor_msgs::PointCloud2 laserCloudCornerLast2;
                pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
                laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudCornerLast2.header.frame_id = "/camera";
                pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

                sensor_msgs::PointCloud2 laserCloudSurfLast2;
                pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
                laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudSurfLast2.header.frame_id = "/camera";
                pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

                systemInited = true;
                std::cout<< "Initialization finished \n";
                continue;
            }

            // if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100)
            // {
                int cornerPointsSharpNum = cornerPointsSharp->points.size();
                int surfPointsFlatNum = surfPointsFlat->points.size();
                
                para_q[0] = 0;
                para_q[1] = 0;
                para_q[2] = 0;
                para_q[3] = 1;
                para_t[0] = 0;
                para_t[1] = 0;
                para_t[2] = 0;
                
                for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter)
                {
                    // LOG(INFO) << "******************************* " << opti_counter << " ***************************************";
                    // LOG(INFO) << "parameter is " << parameters[0] << ", " << parameters[1] << ", " << parameters[2]
                    //                              << ", " << parameters[3] << ", " << parameters[4] << ", " << parameters[5]
                    //                              << ", " << parameters[6];
                    
                    // LOG(INFO) << "optimization counter is " << opti_counter;
                    //ceres::LossFunction *loss_function = NULL;
                    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.01);
                    ceres::LocalParameterization *q_parameterization = 
                        new ceres::EigenQuaternionParameterization();
                    ceres::Problem::Options problem_options;
                    problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

                    ceres::Problem problem(problem_options);
                    problem.AddParameterBlock(para_q, 4, q_parameterization);
                    problem.AddParameterBlock(para_t, 3);

                    pcl::PointXYZI pointSel;
                    std::vector<int> pointSearchInd;
                    std::vector<float> pointSearchSqDis;

                    /* std::cout << "****************************************************************\n";
                    pcl::PointCloud<PointType>::ConstPtr tmp_corners = kdtreeCornerLast->getInputCloud();
                    for (int ii = 0; ii < tmp_corners->points.size(); ++ii)
                    {
                        std::cout << tmp_corners->points[ii].x << ", "
                                    << tmp_corners->points[ii].y << ", "
                                    << tmp_corners->points[ii].z << ", "
                                    << tmp_corners->points[ii].intensity << '\n';
                    }
                    std::cout << "****************************************************************\n"; */

                    // initial guess
                    if(0)
                    {
                        Eigen::Matrix3d rela_R_gt;
                        Eigen::Vector3d rela_t_gt;
                        getRelativeGTPose(timeCornerPointsSharp + 0.1, timeCornerPointsSharp, rela_R_gt, rela_t_gt);
                        Eigen::Quaterniond rela_q_gt(rela_R_gt);
                        //printf("gt relative q %f %f %f %f t %f %f %f \n", rela_q_gt.w(), rela_q_gt.x(), rela_q_gt.y(), rela_q_gt.z(),
                        //                                                  rela_t_gt.x(), rela_t_gt.y(), rela_t_gt.z());
                        para_q[0] = rela_q_gt.x();
                        para_q[1] = rela_q_gt.y();
                        para_q[2] = rela_q_gt.z();
                        para_q[3] = rela_q_gt.w();
                        para_t[0] = rela_t_gt.x();
                        para_t[1] = rela_t_gt.y();
                        para_t[2] = rela_t_gt.z();
                        printf("init guess q %f %f %f %f t %f %f %f \n", para_q[0], para_q[1], para_q[2], para_q[3],
                                                                          para_t[0], para_t[1], para_t[2]);
                    
                    }


                    // find correspondence for corner features
                    for (size_t i = 0; i < cornerPointsSharpNum; ++i)
                    {
                        TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);
                        //pointSel = cornerPointsSharp->points[i];
                        // std::cout << "corner-" << i << ":" << cornerPointsSharp->points[i].x << ", " 
                        //           << cornerPointsSharp->points[i].y << ", " << cornerPointsSharp->points[i].z
                        //           << ", undistort result: " << pointSel.x << ", " << pointSel.y << ", " << pointSel.z << '\n';

                        kdtreeCornerLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        int closestPointInd = -1, minPointInd2 = -1;
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            closestPointInd = pointSearchInd[0];
                            int closestPointScanID = int(laserCloudCornerLast->points[closestPointInd].intensity);

                            // std::cout << "corner-" << i << ", index " << closestPointInd << ", dis " << pointSearchSqDis[0] 
                            //           << ", closest point scan ID " << closestPointScanID << '\n';
                            // std::cout << "closest point is " << laserCloudCornerLast->points[closestPointInd].x << ", "
                            //           << laserCloudCornerLast->points[closestPointInd].y << ", "
                            //           << laserCloudCornerLast->points[closestPointInd].z << ", "
                            //           << laserCloudCornerLast->points[closestPointInd].intensity << '\n';
                            

                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD;
                            // search in the direction of increasing scan line
                            for (int j = closestPointInd+1; j < laserCloudCornerLast->points.size(); ++j)
                            {
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) <= closestPointScanID)
                                    continue;

                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;
                                
                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) * 
                                                    (laserCloudCornerLast->points[j].x - pointSel.x) + 
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) * 
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) + 
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) * 
                                                    (laserCloudCornerLast->points[j].z - pointSel.z);
                                // std::cout << "pointSqDis " << pointSqDis << ", and scan is " << int(laserCloudCornerLast->points[j].intensity) << '\n';
                                
                                if (pointSqDis < minPointSqDis2)
                                {
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }

                            // search in the direction of decreasing scan line
                            for (int j = closestPointInd-1; j >= 0; --j)
                            {
                                // if in the same scan line, continue
                                if (int(laserCloudCornerLast->points[j].intensity) >= closestPointScanID)
                                    continue;
                                
                                // if not in nearby scans, end the loop
                                if (int(laserCloudCornerLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;
                                
                                double pointSqDis = (laserCloudCornerLast->points[j].x - pointSel.x) * 
                                                    (laserCloudCornerLast->points[j].x - pointSel.x) + 
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) * 
                                                    (laserCloudCornerLast->points[j].y - pointSel.y) + 
                                                    (laserCloudCornerLast->points[j].z - pointSel.z) * 
                                                    (laserCloudCornerLast->points[j].z - pointSel.z);

                                if (pointSqDis < minPointSqDis2)
                                {
                                    // find nearer point
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                            }
                        }
                        if (minPointInd2 >= 0) // both closestPointInd and minPointInd2 is valid
                        {
                            // std::cout << "corner-" << i << ", correspondence is " << closestPointInd
                            //           << " and " << minPointInd2 << '\n';
                            Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                                       cornerPointsSharp->points[i].y,
                                                       cornerPointsSharp->points[i].z);
                            Eigen::Vector3d last_point_a(laserCloudCornerLast->points[closestPointInd].x,
                                                         laserCloudCornerLast->points[closestPointInd].y,
                                                         laserCloudCornerLast->points[closestPointInd].z);
                            Eigen::Vector3d last_point_b(laserCloudCornerLast->points[minPointInd2].x,
                                                         laserCloudCornerLast->points[minPointInd2].y,
                                                         laserCloudCornerLast->points[minPointInd2].z);
                            // ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<LidarFactor, 1, 4, 3>(
                            //     new LidarFactor(curr_point, last_point_a, last_point_b, distortion_ratio, i));

                            //LidarFactor *lidar_factor = new LidarFactor(curr_point, last_point_a, last_point_b, distortion_ratio);
                            //problem.AddResidualBlock(lidar_factor, loss_function, parameters, parameters+4);

                            double s;
                            if (DISTORTION)
                                s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) / SCAN_PERIOD;
                            else
                                s = 1.0;
                            ceres::CostFunction* cost_function = LidarEdgeFactor::Create(curr_point, last_point_a, last_point_b, s);
                            problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                            corner_correspondence++;
                            
                            if(closestPointInd == minPointInd2)
                                ROS_BREAK();

                            if((last_point_a[0] == last_point_b[0]) &&
                               (last_point_a[1] == last_point_b[1]) &&
                               (last_point_a[2] == last_point_b[2]))
                                ROS_BREAK();
                            
                            if(0)
                            {
                                std::cout << "point is " << curr_point[0] << ", " << curr_point[1] << ", " << curr_point[2] << ", and point_a is " 
                                       << last_point_a[0] << ", " << last_point_a[1] << ", " << last_point_a[2] << ", and point_b is " 
                                       << last_point_b[0] << ", " << last_point_b[1] << ", " << last_point_b[2] << '\n';
                               double **para = new double *[2];
                               para[0] = para_q;
                               para[1] = para_t;
                               double *res = new double[3];
                               double **jaco = new double *[2];
                               jaco[0] = new double[3 * 4];
                               jaco[1] = new double[3 * 3];
                               cost_function->Evaluate(para, res, jaco);
                               printf("error %f %f %f \n", res[0], res[1], res[2]); 
                               std::cout << Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(jaco[0]) << std::endl;
                               std::cout << Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(jaco[1]) << std::endl;

                               Eigen::Quaterniond q{para_q[3], para_q[0], para_q[1], para_q[2]};
                               Eigen::Vector3d t{para_t[0], para_t[1], para_t[2]};
                               Eigen::Vector3d t_cur;
                               t_cur = q.inverse() * (curr_point - t);
                               double e;
                               e = ((t_cur - last_point_a).cross((t_cur - last_point_b))).norm() / (last_point_a - last_point_b).norm();
                               printf("error my %f \n", e);
                            }
                           
                           
                        }
                    }

                    // find correspondence for plane features
                    if(1)
                    {
                    for (size_t i = 0; i < surfPointsFlatNum; ++i)
                    {
                        TransformToStart(&(surfPointsFlat->points[i]), &pointSel);

                        // std::cout << "plane-" << i << ":" << surfPointsFlat->points[i].x << ", " 
                        //           << surfPointsFlat->points[i].y << ", " << surfPointsFlat->points[i].z
                        //           << ", undistort result: " << pointSel.x << ", " << pointSel.y << ", " << pointSel.z << '\n';

                        kdtreeSurfLast->nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);

                        int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
                        if (pointSearchSqDis[0] < DISTANCE_SQ_THRESHOLD)
                        {
                            closestPointInd = pointSearchInd[0];
                            
                            // get closest point's scan ID
                            int closestPointScanID = int(laserCloudSurfLast->points[closestPointInd].intensity);
                            double minPointSqDis2 = DISTANCE_SQ_THRESHOLD, minPointSqDis3 = DISTANCE_SQ_THRESHOLD;
                            
                            // search in the direction of increasing scan line
                            for (int j = closestPointInd+1; j < laserCloudSurfLast->points.size(); ++j)
                            {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) > (closestPointScanID + NEARBY_SCAN))
                                    break;
                                
                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) * 
                                                    (laserCloudSurfLast->points[j].x - pointSel.x) + 
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) * 
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) + 
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) * 
                                                    (laserCloudSurfLast->points[j].z - pointSel.z);
                                
                                // if in the same or lower scan line
                                if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                // if in the higher scan line
                                else if (int(laserCloudSurfLast->points[j].intensity) > closestPointScanID && pointSqDis < minPointSqDis3)  
                                {
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }

                            // search in the direction of decreasing scan line
                            for (int j = closestPointInd-1; j >= 0; --j)
                            {
                                // if not in nearby scans, end the loop
                                if (int(laserCloudSurfLast->points[j].intensity) < (closestPointScanID - NEARBY_SCAN))
                                    break;
                                
                                double pointSqDis = (laserCloudSurfLast->points[j].x - pointSel.x) * 
                                                    (laserCloudSurfLast->points[j].x - pointSel.x) + 
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) * 
                                                    (laserCloudSurfLast->points[j].y - pointSel.y) + 
                                                    (laserCloudSurfLast->points[j].z - pointSel.z) * 
                                                    (laserCloudSurfLast->points[j].z - pointSel.z);
                                
                                // if in the same or higher scan line
                                if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScanID && pointSqDis < minPointSqDis2)
                                {
                                    minPointSqDis2 = pointSqDis;
                                    minPointInd2 = j;
                                }
                                else if (int(laserCloudSurfLast->points[j].intensity) < closestPointScanID && pointSqDis < minPointSqDis3)
                                {
                                    // find nearer point
                                    minPointSqDis3 = pointSqDis;
                                    minPointInd3 = j;
                                }
                            }

                            if (minPointInd2 >= 0 && minPointInd3 >= 0)
                            {
                                // std::cout << "plane-" << i << ", correspondence is " << closestPointInd
                                //           << " and " << minPointInd2 << " and " << minPointInd3 << '\n';

                                Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                                           surfPointsFlat->points[i].y,
                                                           surfPointsFlat->points[i].z);
                                Eigen::Vector3d last_point_a(laserCloudSurfLast->points[closestPointInd].x,
                                                             laserCloudSurfLast->points[closestPointInd].y,
                                                             laserCloudSurfLast->points[closestPointInd].z);
                                Eigen::Vector3d last_point_b(laserCloudSurfLast->points[minPointInd2].x,
                                                             laserCloudSurfLast->points[minPointInd2].y,
                                                             laserCloudSurfLast->points[minPointInd2].z);
                                Eigen::Vector3d last_point_c(laserCloudSurfLast->points[minPointInd3].x,
                                                             laserCloudSurfLast->points[minPointInd3].y,
                                                             laserCloudSurfLast->points[minPointInd3].z);
                                // ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<LidarFactor, 1, 4, 3>(
                                //     new LidarFactor(curr_point, last_point_a, last_point_b, last_point_c, distortion_ratio, i));
                                //LidarFactor *lidar_factor = new LidarFactor(curr_point, last_point_a, last_point_b, last_point_c, distortion_ratio);
                                //problem.AddResidualBlock(lidar_factor, loss_function, parameters, parameters+4);
                                
                                double s;
                                if(DISTORTION)
                                    s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) / SCAN_PERIOD;
                                else
                                    s = 1.0;
                                ceres::CostFunction* cost_function = LidarPlaneFactor::Create(curr_point, last_point_a, last_point_b, last_point_c, s);
                                problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                                plane_correspondence++;
                                
                                if(0)
                                {
                                    std::cout << "point is " << curr_point[0] << ", " << curr_point[1] << ", " << curr_point[2] << ", and point_a is " 
                                               << last_point_a[0] << ", " << last_point_a[1] << ", " << last_point_a[2] << ", and point_b is " 
                                               << last_point_b[0] << ", " << last_point_b[1] << ", " << last_point_b[2] << ", and point_c is " 
                                               << last_point_c[0] << ", " << last_point_c[1] << ", " << last_point_c[2] << '\n';

                                    double **para = new double *[2];
                                    para[0] = para_q;
                                    para[1] = para_t;
                                    double *res = new double[1];
                                    double **jaco = new double *[2];
                                    jaco[0] = new double[1 * 4];
                                    jaco[1] = new double[1 * 3];
                                    cost_function->Evaluate(para, res, jaco);
                                    printf("plane error %f \n", res[0]); 
                                    std::cout << Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>>(jaco[0]) << std::endl;
                                    std::cout << Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>>(jaco[1]) << std::endl;

                                    Eigen::Quaterniond q{para_q[3], para_q[0], para_q[1], para_q[2]};
                                    Eigen::Vector3d t{para_t[0], para_t[1], para_t[2]};
                                    Eigen::Vector3d t_cur;
                                    t_cur = q.inverse() * (curr_point - t);
                                    double e;
                                    Eigen::Vector3d jlm_cross = (last_point_a - last_point_b).cross(last_point_a - last_point_c);
                                    e = (t_cur - last_point_a).dot(jlm_cross) / jlm_cross.norm();
                                    printf("plane error my %f \n", e);
                                }
                                
                            }
                        }
                    }
                    }

                    // std::cout << "parameters: " << parameters[0] << ", " << parameters[1] << ", " << parameters[2]
                    //           << ", " << parameters[3] << ", " << parameters[4] << ", " << parameters[5] << ", "
                    //           << parameters[6] << '\n';

                    if ((corner_correspondence + plane_correspondence) < 10)
                    {
                        printf("less correspondence! *************************************************\n");
                        break;
                    }

                    ceres::Solver::Options options;
                    options.linear_solver_type = ceres::DENSE_QR;
                    options.max_num_iterations = 5;
                    //options.check_gradients = false;
                    //options.gradient_check_relative_precision = 1e-4;
                    options.minimizer_progress_to_stdout = true;
                    ceres::Solver::Summary summary;
                    ceres::Solve(options, &problem, &summary);

                    LOG(INFO) << summary.BriefReport();
                    LOG(INFO) << "parameter is " << para_t[0] << ", " << para_t[1] << ", " << para_t[2]
                                                 << ", " << para_q[3] << ", " << para_q[0] << ", " << para_q[1]
                                                 << ", " << para_q[2];
                    LOG(INFO) << "corner : " << corner_correspondence << ", and plane : " << plane_correspondence << '\n';
                    corner_correspondence = plane_correspondence = 0;


                }

                t_w_curr = t_w_curr + q_w_curr * t_last_curr;
                q_w_curr = q_w_curr * q_last_curr;

            // }    // if (cornerNum && planeNum)
                {
                    printf("finish one message %f\n", timeCornerPointsSharp);

                    printf("\n \n");
                    printf("my q %f %f %f %f t %f %f %f \n",q_w_curr.w(), q_w_curr.x(), q_w_curr.y(), q_w_curr.z(), 
                                                         t_w_curr.x(), t_w_curr.y(), t_w_curr.z());
                    Eigen::Matrix3d R_gt;
                    Eigen::Vector3d t_gt;
                    getGTPose(timeCornerPointsSharp + 0.1, R_gt, t_gt);
                    Eigen::Quaterniond q_gt(R_gt);

                    printf("gt q %f %f %f %f t %f %f %f \n",q_gt.w(), q_gt.x(), q_gt.y(), q_gt.z(), 
                                                            t_gt.x(), t_gt.y(), t_gt.z());

                    printf("\n");
                    printf("my relative q %f %f %f %f t %f %f %f \n", para_q[3], para_q[0], para_q[1], para_q[2],
                                                                      para_t[0], para_t[1], para_t[2]);

                    Eigen::Matrix3d rela_R_gt;
                    Eigen::Vector3d rela_t_gt;
                    getRelativeGTPose(timeCornerPointsSharp + 0.1, timeCornerPointsSharp, rela_R_gt, rela_t_gt);
                    Eigen::Quaterniond rela_q_gt(rela_R_gt);
                    printf("gt relative q %f %f %f %f t %f %f %f \n", rela_q_gt.w(), rela_q_gt.x(), rela_q_gt.y(), rela_q_gt.z(),
                                                                      rela_t_gt.x(), rela_t_gt.y(), rela_t_gt.z());

                    printf("\n \n");
                }


                // set gt
                if(0)
                {
                    //printf("\n");
                    //printf("my relative q %f %f %f %f t %f %f %f \n", parameters[3], parameters[0], parameters[1], parameters[2],
                    //                                                  parameters[4], parameters[5], parameters[6]);

                    Eigen::Matrix3d rela_R_gt;
                    Eigen::Vector3d rela_t_gt;
                    getRelativeGTPose(timeCornerPointsSharp + 0.1, timeCornerPointsSharp, rela_R_gt, rela_t_gt);
                    Eigen::Quaterniond rela_q_gt(rela_R_gt);
                    //printf("gt relative q %f %f %f %f t %f %f %f \n", rela_q_gt.w(), rela_q_gt.x(), rela_q_gt.y(), rela_q_gt.z(),
                    //                                                  rela_t_gt.x(), rela_t_gt.y(), rela_t_gt.z());
                    
                    para_q[0] = rela_q_gt.x();
                    para_q[1] = rela_q_gt.y();
                    para_q[2] = rela_q_gt.z();
                    para_q[3] = rela_q_gt.w();
                    para_t[0] = rela_t_gt.x();
                    para_t[1] = rela_t_gt.y();
                    para_t[2] = rela_t_gt.z();
                    printf("set gt q %f %f %f %f t %f %f %f \n", para_q[0], para_q[1], para_q[2], para_q[3],
                                                                      para_t[0], para_t[1], para_t[2]);
                    
                }

            // publish point cloud for visualization    
            {
                // publish corner points
                pcl::PointCloud<PointType> cornerPointsCurrToLast;
                int cornerPointsSharpNum = cornerPointsSharp->points.size();
                for (int i = 0; i < cornerPointsSharpNum; i++) 
                {
                    pcl::PointXYZI pointSel;
                    TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);
                    cornerPointsCurrToLast.push_back(pointSel);
                }
                sensor_msgs::PointCloud2 currToLastPointCloud;
                pcl::toROSMsg(cornerPointsCurrToLast, currToLastPointCloud);
                currToLastPointCloud.header.stamp = ros::Time().fromSec(timeCornerPointsSharp);
                currToLastPointCloud.header.frame_id = "/camera_init";
                pubCurrToLastPointCloud.publish(currToLastPointCloud);


                sensor_msgs::PointCloud2 lastPointCloud;
                pcl::toROSMsg(*laserCloudCornerLast, lastPointCloud);
                lastPointCloud.header.stamp = ros::Time().fromSec(timeCornerPointsSharp);
                lastPointCloud.header.frame_id = "/camera_init";
                pubLastPointCloud.publish(lastPointCloud);

                // publish plane points
                pcl::PointCloud<PointType> planePointsCurrToLast;
                int surfPointsFlatNum = surfPointsFlat->points.size();
                for (int i = 0; i < surfPointsFlatNum; i++) 
                {
                    pcl::PointXYZI pointSel;
                    TransformToStart(&(surfPointsFlat->points[i]), &pointSel);
                    planePointsCurrToLast.push_back(pointSel);
                }
                sensor_msgs::PointCloud2 currToLastPlanePointCloud;
                pcl::toROSMsg(planePointsCurrToLast, currToLastPlanePointCloud);
                currToLastPlanePointCloud.header.stamp = ros::Time().fromSec(timeCornerPointsSharp);
                currToLastPlanePointCloud.header.frame_id = "/camera_init";
                pubCurrToLastPlanePointCloud.publish(currToLastPlanePointCloud);


                sensor_msgs::PointCloud2 lastPlanePointCloud;
                pcl::toROSMsg(*laserCloudSurfLast, lastPlanePointCloud);
                lastPlanePointCloud.header.stamp = ros::Time().fromSec(timeCornerPointsSharp);
                lastPlanePointCloud.header.frame_id = "/camera_init";
                pubLastPlanePointCloud.publish(lastPlanePointCloud);
            }

            // std::cout << t_w_curr.x() << ", " << t_w_curr.y() << ", " << t_w_curr.z() << '\n';

            // publish odometry
            laserOdometry.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
            laserOdometry.pose.pose.orientation.x = q_w_curr.x();
            laserOdometry.pose.pose.orientation.y = q_w_curr.y();
            laserOdometry.pose.pose.orientation.z = q_w_curr.z();
            laserOdometry.pose.pose.orientation.w = q_w_curr.w();
            laserOdometry.pose.pose.position.x = t_w_curr.x();
            laserOdometry.pose.pose.position.y = t_w_curr.y();
            laserOdometry.pose.pose.position.z = t_w_curr.z();
            pubLaserOdometry.publish(laserOdometry);

            geometry_msgs::PoseStamped laserPose;
            laserPose.header = laserOdometry.header;
            laserPose.pose = laserOdometry.pose.pose;
            laserPath.header.stamp = laserOdometry.header.stamp;
            laserPath.poses.push_back(laserPose);
            pubLaserPath.publish(laserPath);

            // brodecast new coordinate
            laserOdometryTrans.stamp_ = ros::Time().fromSec(timeSurfPointsLessFlat);
            laserOdometryTrans.setRotation(tf::Quaternion(q_w_curr.x(), q_w_curr.y(), q_w_curr.z(), q_w_curr.w()));
            laserOdometryTrans.setOrigin(tf::Vector3(t_w_curr.x(), t_w_curr.y(), t_w_curr.z()));
            tfBroadcaster.sendTransform(laserOdometryTrans);

            // transform corner features and plane features to the scan end point
            int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
            for (int i = 0; i < cornerPointsLessSharpNum; i++) 
            {
                TransformToEnd(&cornerPointsLessSharp->points[i], &cornerPointsLessSharp->points[i]);
            }

            int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
            for (int i = 0; i < surfPointsLessFlatNum; i++) 
            {
                TransformToEnd(&surfPointsLessFlat->points[i], &surfPointsLessFlat->points[i]);
            }

            frameCount++;
            // all points in the cloud, undistort every `skipFrameNum` frame
            if (frameCount >= skipFrameNum + 1) 
            {
                int laserCloudFullResNum = laserCloudFullRes->points.size();
                for (int i = 0; i < laserCloudFullResNum; i++) 
                {
                    TransformToEnd(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
                }
            }

            pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
            cornerPointsLessSharp = laserCloudCornerLast;
            laserCloudCornerLast = laserCloudTemp;

            laserCloudTemp = surfPointsLessFlat;
            surfPointsLessFlat = laserCloudSurfLast;
            laserCloudSurfLast = laserCloudTemp;

            laserCloudCornerLastNum = laserCloudCornerLast->points.size();
            laserCloudSurfLastNum = laserCloudSurfLast->points.size();

            // std::cout << "the size of corner last is " << laserCloudCornerLastNum << ", and the size of surf last is " << laserCloudSurfLastNum << '\n';

            // TODO
            // if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) 
            // {
                printf("laserCloudCornerLast size %d\n", laserCloudCornerLast->points.size());
                kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
                printf("laserCloudSurfLast size %d\n", laserCloudSurfLast->points.size());
                kdtreeSurfLast->setInputCloud(laserCloudSurfLast);
            // }

            // clear parameters
            // parameters[0] = 1;
            // parameters[1] = 0;
            // parameters[2] = 0;
            // parameters[3] = 0;
            // parameters[4] = 0;
            // parameters[5] = 0;
            // parameters[6] = 0;

            // send corner features, plane features and all points to laserMapping (every `skipFrameNum` frames)
            if (frameCount >= skipFrameNum + 1) 
            {
                frameCount = 0;

                sensor_msgs::PointCloud2 laserCloudCornerLast2;
                pcl::toROSMsg(*laserCloudCornerLast, laserCloudCornerLast2);
                laserCloudCornerLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudCornerLast2.header.frame_id = "/camera";
                pubLaserCloudCornerLast.publish(laserCloudCornerLast2);

                sensor_msgs::PointCloud2 laserCloudSurfLast2;
                pcl::toROSMsg(*laserCloudSurfLast, laserCloudSurfLast2);
                laserCloudSurfLast2.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudSurfLast2.header.frame_id = "/camera";
                pubLaserCloudSurfLast.publish(laserCloudSurfLast2);

                sensor_msgs::PointCloud2 laserCloudFullRes3;
                pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
                laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeSurfPointsLessFlat);
                laserCloudFullRes3.header.frame_id = "/camera";
                pubLaserCloudFullRes.publish(laserCloudFullRes3);
            }
        }
        rate.sleep();
    }
    return 0;
}