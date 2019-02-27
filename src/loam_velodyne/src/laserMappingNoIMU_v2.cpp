#include <math.h>
#include <vector>
#include <loam_velodyne/common.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv/cv.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>

#include "lidarFactor_v6.cpp"
#include "loam_velodyne/common.h"
#include "loam_velodyne/tic_toc.h"

#define BACKWARD_HAS_DW 1
#include <backward.hpp>
namespace backward
{
backward::SignalHandling sh;
} // namespace backward

//扫描周期
const float scanPeriod = 0.1;

//控制接收到的点云数据，每隔几帧处理一次
const int stackFrameNum = 1;
//控制处理得到的点云map，每隔几次publich给rviz显示
const int mapFrameNum = 5;

//时间戳
double timeLaserCloudCornerLast = 0;
double timeLaserCloudSurfLast = 0;
double timeLaserCloudFullRes = 0;
double timeLaserOdometry = 0;

//接收标志
bool newLaserCloudCornerLast = false;
bool newLaserCloudSurfLast = false;
bool newLaserCloudFullRes = false;
bool newLaserOdometry = false;

int laserCloudCenWidth = 10;
int laserCloudCenHeight = 10;
int laserCloudCenDepth = 5;
const int laserCloudWidth = 21;
const int laserCloudHeight = 21;
const int laserCloudDepth = 11;
//点云方块集合最大数量
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth; //4851

//lidar视域范围内(FOV)的点云集索引
int laserCloudValidInd[125];
//lidar周围的点云集索引
int laserCloudSurroundInd[125];

//最新接收到的边沿点
pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(new pcl::PointCloud<PointType>());
//最新接收到的平面点
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(new pcl::PointCloud<PointType>());
//存放当前收到的下采样之后的边沿点(in the local frame)
pcl::PointCloud<PointType>::Ptr laserCloudCornerStack(new pcl::PointCloud<PointType>());
//存放当前收到的下采样之后的平面点(in the local frame)
pcl::PointCloud<PointType>::Ptr laserCloudSurfStack(new pcl::PointCloud<PointType>());
//匹配使用的特征点（下采样之后的）
pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());
//map中提取的匹配使用的边沿点
pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>());
//map中提取的匹配使用的平面点
pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>());
//点云全部点
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
//array都是以50米为单位的立方体地图，运行过程中会一直保存(有需要的话可考虑优化，只保存近邻的，或者直接数组开小一点)
//存放边沿点的cube
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray[laserCloudNum];
//存放平面点的cube
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray[laserCloudNum];
//中间变量，存放下采样过的边沿点
pcl::PointCloud<PointType>::Ptr laserCloudCornerArray2[laserCloudNum];
//中间变量，存放下采样过的平面点
pcl::PointCloud<PointType>::Ptr laserCloudSurfArray2[laserCloudNum];

//kd-tree
pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());

Eigen::Quaterniond q_w_curr_tilde(1, 0, 0, 0);
Eigen::Vector3d t_w_curr_tilde(0, 0, 0);

double parameters[7] = {0, 0, 0, 1, 0, 0, 0};
Eigen::Map<Eigen::Quaterniond> q_w_curr(parameters);
Eigen::Map<Eigen::Vector3d> t_w_curr(parameters + 4);

Eigen::Quaterniond q_w_last_tilde(1, 0, 0, 0);
Eigen::Vector3d t_w_last_tilde(0, 0, 0);

Eigen::Quaterniond q_w_last(1, 0, 0, 0);
Eigen::Vector3d t_w_last(0, 0, 0);

#define TIME_STILL 1

inline Eigen::Matrix3d RPY2RotationMatrix(double roll, double pitch, double yaw)
{
	return (Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
			Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))
		.toRotationMatrix();
}

void getGTPose(double timestamp, Eigen::Matrix3d &R_w_curr, Eigen::Vector3d &t_w_curr)
{
	double LINE_VELOCITY = 0.2; // velocity: 0.2 m/s
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

		R_w_curr = RPY2RotationMatrix(ROLL_VELOCITY * move_time, PITCH_VELOCITY * move_time, YAW_VELOCITY * move_time);
		t_w_curr << move_time * LINE_VELOCITY, move_time * LINE_VELOCITY, sin(10 * move_time * LINE_VELOCITY);
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

//基于匀速模型，根据上次微调的结果和odometry这次与上次计算的结果，猜测一个新的世界坐标系的转换矩阵transformTobeMapped
void transformAssociateToMap()
{
	Eigen::Matrix4d T_w_curr, T_w_last, T_w_last_tilde, T_w_curr_tilde;

	T_w_last = Eigen::Matrix4d::Identity();
	T_w_last.block<3, 3>(0, 0) = q_w_last.toRotationMatrix();
	T_w_last.block<3, 1>(0, 3) = t_w_last;

	T_w_last_tilde = Eigen::Matrix4d::Identity();
	T_w_last_tilde.block<3, 3>(0, 0) = q_w_last_tilde.toRotationMatrix();
	T_w_last_tilde.block<3, 1>(0, 3) = t_w_last_tilde;

	T_w_curr_tilde = Eigen::Matrix4d::Identity();
	T_w_curr_tilde.block<3, 3>(0, 0) = q_w_curr_tilde.toRotationMatrix();
	T_w_curr_tilde.block<3, 1>(0, 3) = t_w_curr_tilde;

	T_w_curr = T_w_last * T_w_last_tilde.inverse() * T_w_curr_tilde;

	q_w_curr = T_w_curr.block<3, 3>(0, 0);
	t_w_curr = T_w_curr.block<3, 1>(0, 3);
}

//记录odometry发送的转换矩阵与mapping之后的转换矩阵，下一帧点云会使用(有IMU的话会使用IMU进行补偿)
void transformUpdate()
{
	q_w_last_tilde = q_w_curr_tilde;
	t_w_last_tilde = t_w_curr_tilde;
	q_w_last = q_w_curr;
	t_w_last = t_w_curr;
}

//根据调整计算后的转移矩阵，将点注册到全局世界坐标系下
void pointAssociateToMap(PointType const *const pi, PointType *const po)
{
	Eigen::Vector3d point_curr(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_w = q_w_curr * point_curr + t_w_curr;
	po->x = point_w.x();
	po->y = point_w.y();
	po->z = point_w.z();
	po->intensity = pi->intensity;
}

//点转移到局部坐标系下
void pointAssociateTobeMapped(PointType const *const pi, PointType *const po)
{
	Eigen::Vector3d point_w(pi->x, pi->y, pi->z);
	Eigen::Vector3d point_curr = q_w_curr.inverse() * (point_w - t_w_curr);
	po->x = point_curr.x();
	po->y = point_curr.y();
	po->z = point_curr.z();
	po->intensity = pi->intensity;
}

//接收边沿点
void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudCornerLast2)
{
	timeLaserCloudCornerLast = laserCloudCornerLast2->header.stamp.toSec();

	laserCloudCornerLast->clear();
	pcl::fromROSMsg(*laserCloudCornerLast2, *laserCloudCornerLast);

	newLaserCloudCornerLast = true;
}

//接收平面点
void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudSurfLast2)
{
	timeLaserCloudSurfLast = laserCloudSurfLast2->header.stamp.toSec();

	laserCloudSurfLast->clear();
	pcl::fromROSMsg(*laserCloudSurfLast2, *laserCloudSurfLast);

	newLaserCloudSurfLast = true;
}

//接收点云全部点
void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudFullRes2)
{
	timeLaserCloudFullRes = laserCloudFullRes2->header.stamp.toSec();

	laserCloudFullRes->clear();
	pcl::fromROSMsg(*laserCloudFullRes2, *laserCloudFullRes);

	newLaserCloudFullRes = true;
}

//接收旋转平移信息
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)
{
	timeLaserOdometry = laserOdometry->header.stamp.toSec();

	q_w_curr_tilde.x() = laserOdometry->pose.pose.orientation.x;
	q_w_curr_tilde.y() = laserOdometry->pose.pose.orientation.y;
	q_w_curr_tilde.z() = laserOdometry->pose.pose.orientation.z;
	q_w_curr_tilde.w() = laserOdometry->pose.pose.orientation.w;
	t_w_curr_tilde.x() = laserOdometry->pose.pose.position.x;
	t_w_curr_tilde.y() = laserOdometry->pose.pose.position.y;
	t_w_curr_tilde.z() = laserOdometry->pose.pose.position.z;

	newLaserOdometry = true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laserMapping");
	ros::NodeHandle nh;

	ros::Subscriber subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 100, laserCloudCornerLastHandler);

	ros::Subscriber subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 100, laserCloudSurfLastHandler);

	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 100, laserOdometryHandler);

	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_cloud_3", 100, laserCloudFullResHandler);

	ros::Publisher pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 100);

	ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/velodyne_cloud_registered", 100);

	ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 100);
	nav_msgs::Odometry odomAftMapped;
	odomAftMapped.header.frame_id = "/camera_init";
	odomAftMapped.child_frame_id = "/aft_mapped";

	tf::TransformBroadcaster tfBroadcaster;
	tf::StampedTransform aftMappedTrans;
	aftMappedTrans.frame_id_ = "/camera_init";
	aftMappedTrans.child_frame_id_ = "/aft_mapped";

	ros::Publisher pubLaserAfterMappedPath = nh.advertise<nav_msgs::Path>("/aft_mapped_path", 5);
	nav_msgs::Path laserAfterMappedPath;
	laserAfterMappedPath.header.frame_id = "/camera_init";

	std::vector<int> pointSearchInd;
	std::vector<float> pointSearchSqDis;

	PointType pointOri, pointSel;

	//创建VoxelGrid滤波器（体素栅格滤波器）
	pcl::VoxelGrid<PointType> downSizeFilterCorner;
	//设置体素大小
	downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);

	pcl::VoxelGrid<PointType> downSizeFilterSurf;
	downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);

	pcl::VoxelGrid<PointType> downSizeFilterMap;
	downSizeFilterMap.setLeafSize(0.6, 0.6, 0.6);

	//指针初始化
	for (int i = 0; i < laserCloudNum; i++)
	{
		laserCloudCornerArray[i].reset(new pcl::PointCloud<PointType>());
		laserCloudSurfArray[i].reset(new pcl::PointCloud<PointType>());
		laserCloudCornerArray2[i].reset(new pcl::PointCloud<PointType>());
		laserCloudSurfArray2[i].reset(new pcl::PointCloud<PointType>());
	}

	int frameCount = stackFrameNum - 1;  //0
	int mapFrameCount = mapFrameNum - 1; //4
	ros::Rate rate(100);
	bool status = ros::ok();
	while (status)
	{
		ros::spinOnce();

		if (newLaserCloudCornerLast && newLaserCloudSurfLast && newLaserCloudFullRes && newLaserOdometry &&
			fabs(timeLaserCloudCornerLast - timeLaserOdometry) < 0.005 &&
			fabs(timeLaserCloudSurfLast - timeLaserOdometry) < 0.005 &&
			fabs(timeLaserCloudFullRes - timeLaserOdometry) < 0.005)
		{
			newLaserCloudCornerLast = false;
			newLaserCloudSurfLast = false;
			newLaserCloudFullRes = false;
			newLaserOdometry = false;

			frameCount++;
			
			TicToc t_whole;
			if (frameCount >= stackFrameNum)
			{

				transformAssociateToMap();

				TicToc t_shift;
				//立方体中点在世界坐标系下的（原点）位置
				//过半取一（以50米进行四舍五入的效果），由于数组下标只能为正数，而地图可能建立在原点前后，因此
				//每一维偏移一个laserCloudCenWidth（该值会动态调整，以使得数组利用最大化，初始值为该维数组长度1/2）的量
				int centerCubeI = int((t_w_curr.x() + 25.0) / 50.0) + laserCloudCenWidth;
				int centerCubeJ = int((t_w_curr.y() + 25.0) / 50.0) + laserCloudCenHeight;
				int centerCubeK = int((t_w_curr.z() + 25.0) / 50.0) + laserCloudCenDepth;

				//由于计算机求余是向零取整，为了不使（-50.0,50.0）求余后都向零偏移，当被求余数为负数时求余结果统一向左偏移一个单位，也即减一
				if (t_w_curr.x() + 25.0 < 0)
					centerCubeI--;
				if (t_w_curr.y() + 25.0 < 0)
					centerCubeJ--;
				if (t_w_curr.z() + 25.0 < 0)
					centerCubeK--;

				//调整之后取值范围:3 < centerCubeI < 18， 3 < centerCubeJ < 18, 3 < centerCubeK < 18
				//如果处于下边界，表明地图向负方向延伸的可能性比较大，则循环移位，将数组中心点向上边界调整一个单位
				while (centerCubeI < 3)
				{
					for (int j = 0; j < laserCloudHeight; j++)
					{
						for (int k = 0; k < laserCloudDepth; k++)
						{ //实现一次循环移位效果
							int i = laserCloudWidth - 1;
							//指针赋值，保存最后一个指针位置
							pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k]; //that's [i + 21 * j + 231 * k]
							pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							//循环移位，I维度上依次后移
							for (; i >= 1; i--)
							{
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCornerArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudSurfArray[i - 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							}
							//将开始点赋值为最后一个点
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeCornerPointer;
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeSurfPointer;
							laserCloudCubeCornerPointer->clear();
							laserCloudCubeSurfPointer->clear();
						}
					}

					centerCubeI++;
					laserCloudCenWidth++;
				}

				//如果处于上边界，表明地图向正方向延伸的可能性比较大，则循环移位，将数组中心点向下边界调整一个单位
				while (centerCubeI >= laserCloudWidth - 3)
				{ //18
					for (int j = 0; j < laserCloudHeight; j++)
					{
						for (int k = 0; k < laserCloudDepth; k++)
						{
							int i = 0;
							pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							//I维度上依次前移
							for (; i < laserCloudWidth - 1; i++)
							{
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCornerArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudSurfArray[i + 1 + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							}
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeCornerPointer;
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeSurfPointer;
							laserCloudCubeCornerPointer->clear();
							laserCloudCubeSurfPointer->clear();
						}
					}

					centerCubeI--;
					laserCloudCenWidth--;
				}

				while (centerCubeJ < 3)
				{
					for (int i = 0; i < laserCloudWidth; i++)
					{
						for (int k = 0; k < laserCloudDepth; k++)
						{
							int j = laserCloudHeight - 1;
							pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							//J维度上，依次后移
							for (; j >= 1; j--)
							{
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCornerArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudSurfArray[i + laserCloudWidth * (j - 1) + laserCloudWidth * laserCloudHeight * k];
							}
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeCornerPointer;
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeSurfPointer;
							laserCloudCubeCornerPointer->clear();
							laserCloudCubeSurfPointer->clear();
						}
					}

					centerCubeJ++;
					laserCloudCenHeight++;
				}

				while (centerCubeJ >= laserCloudHeight - 3)
				{
					for (int i = 0; i < laserCloudWidth; i++)
					{
						for (int k = 0; k < laserCloudDepth; k++)
						{
							int j = 0;
							pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							//J维度上一次前移
							for (; j < laserCloudHeight - 1; j++)
							{
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCornerArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudSurfArray[i + laserCloudWidth * (j + 1) + laserCloudWidth * laserCloudHeight * k];
							}
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeCornerPointer;
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeSurfPointer;
							laserCloudCubeCornerPointer->clear();
							laserCloudCubeSurfPointer->clear();
						}
					}

					centerCubeJ--;
					laserCloudCenHeight--;
				}

				while (centerCubeK < 3)
				{
					for (int i = 0; i < laserCloudWidth; i++)
					{
						for (int j = 0; j < laserCloudHeight; j++)
						{
							int k = laserCloudDepth - 1;
							pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							//K维度上依次后移
							for (; k >= 1; k--)
							{
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k - 1)];
							}
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeCornerPointer;
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeSurfPointer;
							laserCloudCubeCornerPointer->clear();
							laserCloudCubeSurfPointer->clear();
						}
					}

					centerCubeK++;
					laserCloudCenDepth++;
				}

				while (centerCubeK >= laserCloudDepth - 3)
				{
					for (int i = 0; i < laserCloudWidth; i++)
					{
						for (int j = 0; j < laserCloudHeight; j++)
						{
							int k = 0;
							pcl::PointCloud<PointType>::Ptr laserCloudCubeCornerPointer =
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							pcl::PointCloud<PointType>::Ptr laserCloudCubeSurfPointer =
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k];
							//K维度上依次前移
							for (; k < laserCloudDepth - 1; k++)
							{
								laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
								laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
									laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * (k + 1)];
							}
							laserCloudCornerArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeCornerPointer;
							laserCloudSurfArray[i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k] =
								laserCloudCubeSurfPointer;
							laserCloudCubeCornerPointer->clear();
							laserCloudCubeSurfPointer->clear();
						}
					}

					centerCubeK--;
					laserCloudCenDepth--;
				}

				PointType pointOnYAxis;
				pointOnYAxis.x = 0.0;
				pointOnYAxis.y = 0.0;
				pointOnYAxis.z = 10.0;
				//获取y方向上10米高位置的点在世界坐标系下的坐标
				pointAssociateToMap(&pointOnYAxis, &pointOnYAxis);

				int laserCloudValidNum = 0;
				int laserCloudSurroundNum = 0;
				//在每一维附近5个cube(前2个，后2个，中间1个)里进行查找（前后250米范围内，总共500米范围），三个维度总共125个cube
				//在这125个cube里面进一步筛选在视域范围内的cube
				for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++)
				{
					for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++)
					{
						for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++)
						{
							if (i >= 0 && i < laserCloudWidth &&
								j >= 0 && j < laserCloudHeight &&
								k >= 0 && k < laserCloudDepth)
							{ //如果索引合法

								//换算成实际比例，在世界坐标系下的坐标
								float centerX = 50.0 * (i - laserCloudCenWidth);
								float centerY = 50.0 * (j - laserCloudCenHeight);
								float centerZ = 50.0 * (k - laserCloudCenDepth);

								bool isInLaserFOV = false; //判断是否在lidar视线范围的标志（Field of View）
								for (int ii = -1; ii <= 1; ii += 2)
								{
									for (int jj = -1; jj <= 1; jj += 2)
									{
										for (int kk = -1; kk <= 1; kk += 2)
										{
											//上下左右八个顶点坐标
											float cornerX = centerX + 25.0 * ii;
											float cornerY = centerY + 25.0 * jj;
											float cornerZ = centerZ + 25.0 * kk;

											//原点到顶点距离的平方和
											float squaredSide1 = (t_w_curr.x() - cornerX) * (t_w_curr.x() - cornerX) + (t_w_curr.y() - cornerY) * (t_w_curr.y() - cornerY) + (t_w_curr.z() - cornerZ) * (t_w_curr.z() - cornerZ);

											//pointOnYAxis到顶点距离的平方和
											float squaredSide2 = (pointOnYAxis.x - cornerX) * (pointOnYAxis.x - cornerX) + (pointOnYAxis.y - cornerY) * (pointOnYAxis.y - cornerY) + (pointOnYAxis.z - cornerZ) * (pointOnYAxis.z - cornerZ);

											// By the law of cosines, we have a^2 + b^2 - c^2 = 2ab * cos(theta)
											float check1 = 100.0 + squaredSide1 - squaredSide2 - 10.0 * sqrt(3.0) * sqrt(squaredSide1);

											float check2 = 100.0 + squaredSide1 - squaredSide2 + 10.0 * sqrt(3.0) * sqrt(squaredSide1);

											// -60 ~ +60 ???                   //????????????  60 --- 30
											if (check1 < 0 && check2 > 0)
											{ //if |100 + squaredSide1 - squaredSide2| < 10.0 * sqrt(3.0) * sqrt(squaredSide1)
												isInLaserFOV = true;
											}
										}
									}
								}

								//记住视域范围内的cube索引，匹配用
								if (isInLaserFOV)
								{
									laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
									laserCloudValidNum++;
								}
								//记住附近所有cube的索引，显示用
								laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j + laserCloudWidth * laserCloudHeight * k;
								laserCloudSurroundNum++;
							}
						}
					}
				}

				laserCloudCornerFromMap->clear();
				laserCloudSurfFromMap->clear();
				//构建特征点地图，查找匹配使用
				for (int i = 0; i < laserCloudValidNum; i++)
				{
					*laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
					*laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
				}
				int laserCloudCornerFromMapNum = laserCloudCornerFromMap->points.size();
				int laserCloudSurfFromMapNum = laserCloudSurfFromMap->points.size();

				laserCloudCornerStack->clear();
				downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
				downSizeFilterCorner.filter(*laserCloudCornerStack);
				int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

				laserCloudSurfStack->clear();
				downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
				downSizeFilterSurf.filter(*laserCloudSurfStack);
				int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

				printf("map prepare time %f ms\n", t_shift.toc());
				//printf("map corner num %d  surf num %d \n", laserCloudCornerFromMapNum, laserCloudSurfFromMapNum);
				if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 50)
				{
					TicToc t_opt;
					TicToc t_tree;
					kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
					kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);
					printf("build tree time %f ms \n", t_tree.toc());
					if (0)
					{
						printf("\n \n");
						printf("init guess q %f %f %f %f t %f %f %f \n", parameters[3], parameters[0], parameters[1], parameters[2],
							   parameters[4], parameters[5], parameters[6]);
						Eigen::Matrix3d R_gt;
						Eigen::Vector3d t_gt;
						getGTPose(timeLaserOdometry, R_gt, t_gt);
						Eigen::Quaterniond q_gt(R_gt);

						printf("gt q %f %f %f %f t %f %f %f \n", q_gt.w(), q_gt.x(), q_gt.y(), q_gt.z(),
							   t_gt.x(), t_gt.y(), t_gt.z());

						// set initial guess
						/*
              			parameters[0] = q_gt.x();
              			parameters[1] = q_gt.y();
              			parameters[2] = q_gt.z();
              			parameters[3] = q_gt.w();
              			parameters[4] = t_gt.x();
              			parameters[5] = t_gt.y();
              			parameters[6] = t_gt.z();       
              			printf("set real gt initial guess\n");    
              			*/
					}
					
					for (int iterCount = 0; iterCount < 2; iterCount++)
					{

						//ceres::LossFunction *loss_function = NULL;
						ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
						ceres::LocalParameterization *q_parameterization =
							new ceres::EigenQuaternionParameterization();
						ceres::Problem::Options problem_options;
						problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;

						ceres::Problem problem(problem_options);
						problem.AddParameterBlock(parameters, 4, q_parameterization);
						problem.AddParameterBlock(parameters + 4, 3);

						TicToc t_data;
						int corner_num = 0;
						if (1)
						{
							for (int i = 0; i < laserCloudCornerStackNum; i++)
							{
								pointOri = laserCloudCornerStack->points[i];
								//转换回世界坐标系
								pointAssociateToMap(&pointOri, &pointSel);
								kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis); //寻找最近距离五个点

								if (pointSearchSqDis[4] < 1.0)
								{ //5个点中最大距离不超过1才处理
									std::vector<Eigen::Vector3d> nearCorners;
									Eigen::Vector3d center(0, 0, 0);
									for (int j = 0; j < 5; j++)
									{
										Eigen::Vector3d tmp(laserCloudCornerFromMap->points[pointSearchInd[j]].x,
															laserCloudCornerFromMap->points[pointSearchInd[j]].y,
															laserCloudCornerFromMap->points[pointSearchInd[j]].z);
										nearCorners.push_back(tmp);
										center = center + tmp;
									}
									center = center / 5.0;

									Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
									for (int j = 0; j < 5; j++)
									{
										Eigen::Matrix<double, 3, 1> tmpZeroMean = nearCorners[j] - center;
										covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
									}

									//std::cout << "matA1" << std::endl << matA1 << std::endl;
									//std::cout << "covMat" << std::endl << covMat << std::endl;

									//特征值分解
									Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

									// if is indeed line feature
									// note Eigen library sort eigenvalues in increasing order
									if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1])
									{ //如果最大的特征值大于第二大的特征值三倍以上
										/*
										float x0 = pointSel.x;
										float y0 = pointSel.y;
										float z0 = pointSel.z;
										// the first column of matV1 is line direction with norm 1
										// here A(x1, y1, z1) and B(x2, y2, z2) are two points sampled from fitted line
										Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
										Eigen::Vector3d point_on_line(cx, cy, cz);

										Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
										LidarLineFactor *lidar_line_factor = new LidarLineFactor(curr_point, unit_direction, point_on_line);
										problem.AddResidualBlock(lidar_line_factor, loss_function, parameters, parameters+4);
										*/

										Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
										Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
										Eigen::Vector3d point_on_line = center;

										Eigen::Vector3d point_a, point_b;
										point_a = 0.1 * unit_direction + point_on_line;
										point_b = -0.1 * unit_direction + point_on_line;

										ceres::CostFunction *cost_function = LidarEdgeFactor::Create(curr_point, point_a, point_b, 1.0);
										problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
										corner_num++;

										if (0)
										{
											std::cout << "point is " << curr_point[0] << ", " << curr_point[1] << ", " << curr_point[2] << ", and point_a is "
													  << point_a[0] << ", " << point_a[1] << ", " << point_a[2] << ", and point_b is "
													  << point_b[0] << ", " << point_b[1] << ", " << point_b[2] << '\n';
											double **para = new double *[2];
											para[0] = parameters;
											para[1] = parameters + 4;
											double *res = new double[3];
											double **jaco = new double *[2];
											jaco[0] = new double[3 * 4];
											jaco[1] = new double[3 * 3];
											cost_function->Evaluate(para, res, jaco);
											printf("error %f %f %f \n", res[0], res[1], res[2]);
											std::cout << Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>>(jaco[0]) << std::endl;
											std::cout << Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(jaco[1]) << std::endl;

											Eigen::Quaterniond q{parameters[3], parameters[0], parameters[1], parameters[2]};
											Eigen::Vector3d t{parameters[4], parameters[5], parameters[6]};
											Eigen::Vector3d t_cur;
											t_cur = q * curr_point + t;
											Eigen::Vector3d e;
											e = ((t_cur - point_a).cross((t_cur - point_b))) / (point_a - point_b).norm();
											printf("error my %f %f %f\n", e.x(), e.y(), e.z());
										}
									}
								}
							}
						}

						int surf_num = 0;
						if (1)
						{
							for (int i = 0; i < laserCloudSurfStackNum; i++)
							{
								pointOri = laserCloudSurfStack->points[i];
								pointAssociateToMap(&pointOri, &pointSel);
								kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

								Eigen::Matrix<double, 5, 3> matA0;
								Eigen::Matrix<double, 5, 1> matB0 = -1 * Eigen::Matrix<double, 5, 1>::Ones();
								if (pointSearchSqDis[4] < 1.0)
								{
									//构建五个最近点的坐标矩阵
									for (int j = 0; j < 5; j++)
									{
										matA0(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
										matA0(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
										matA0(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
										//printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j, 2));
									}
									//printf(" target pts %f %f %f ", pointSel.x, pointSel.y, pointSel.z);

									//printf("\n");
									//求解matA0*matX0=matB0
									// find the norm of plane
									Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
									double negative_OA_dot_norm = 1 / norm.norm();
									norm.normalize();

									// Here n(pa, pb, pc) is unit norm of plane

									bool planeValid = true;
									for (int j = 0; j < 5; j++)
									{
										// if OX * n > 0.2, then plane is not fit well
										/*
										printf("plane error %f\n", fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
										norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
										norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z + negative_OA_dot_norm));
										*/
										if (fabs(norm(0) * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
												 norm(1) * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
												 norm(2) * laserCloudSurfFromMap->points[pointSearchInd[j]].z + negative_OA_dot_norm) > 0.2)
										{
											planeValid = false;
											break;
										}
									}

									if (planeValid)
									{
										Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);

										/*
										LidarPlaneFactor *lidar_plane_factor = new LidarPlaneFactor(curr_point, norm, negative_OA_dot_norm);
										problem.AddResidualBlock(lidar_plane_factor, loss_function, parameters, parameters+4);
										*/

										ceres::CostFunction *cost_function = LidarPlaneNormFactor::Create(curr_point, norm, negative_OA_dot_norm);
										problem.AddResidualBlock(cost_function, loss_function, parameters, parameters + 4);
										surf_num++;
										if (0)
										{
											std::cout << "point is " << curr_point[0] << ", " << curr_point[1] << ", " << curr_point[2] << " plane norm is " << norm.x() << ", " << norm.y() << ", " << norm.z() << " scalar is " << negative_OA_dot_norm << std::endl;

											double **para = new double *[2];
											para[0] = parameters;
											para[1] = parameters + 4;
											double *res = new double[1];
											double **jaco = new double *[2];
											jaco[0] = new double[1 * 4];
											jaco[1] = new double[1 * 3];
											cost_function->Evaluate(para, res, jaco);
											printf("plane error %f \n", res[0]);
											//std::cout << Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>>(jaco[0]) << std::endl;
											//std::cout << Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>>(jaco[1]) << std::endl;

											Eigen::Quaterniond q{parameters[3], parameters[0], parameters[1], parameters[2]};
											Eigen::Vector3d t{parameters[4], parameters[5], parameters[6]};
											Eigen::Vector3d t_cur;
											t_cur = q * curr_point + t;
											double e;
											e = norm.dot(t_cur) + negative_OA_dot_norm;
											printf("plane error my %f \n", e);
										}
									}
								}
							}
						}

						printf("mapping data assosiation time %f ms \n", t_data.toc());

						TicToc t_solver;
						ceres::Solver::Options options;
						options.linear_solver_type = ceres::DENSE_QR;
						options.max_num_iterations = 5;
						options.minimizer_progress_to_stdout = false;
						options.check_gradients = false;
						options.gradient_check_relative_precision = 1e-4;
						ceres::Solver::Summary summary;
						ceres::Solve(options, &problem, &summary);

						printf("mapping solver time %f ms \n", t_solver.toc());

						//printf("time %f \n", timeLaserOdometry);
						printf("corner factor num %d surf factor num %d\n", corner_num, surf_num);
						//printf("result q %f %f %f %f result t %f %f %f\n", parameters[3], parameters[0], parameters[1], parameters[2],
						//	   parameters[4], parameters[5], parameters[6]);
						if(0)
						{
							Eigen::Matrix3d R_gt;
							Eigen::Vector3d t_gt;
							getGTPose(timeLaserOdometry, R_gt, t_gt);
							Eigen::Quaterniond q_gt(R_gt);

							printf("gt q %f %f %f %f t %f %f %f \n", q_gt.w(), q_gt.x(), q_gt.y(), q_gt.z(),
								   t_gt.x(), t_gt.y(), t_gt.z());
						}
						//printf("\n");
					}
					printf("mapping optimization time %f \n", t_opt.toc());

					//迭代结束更新相关的转移矩阵
					transformUpdate();
				}
				else
				{
					printf("time %f Map corner and surf num are not enough\n", timeLaserOdometry);
				}


				TicToc t_pub;
				//将corner points按距离（比例尺缩小）归入相应的立方体
				for (int i = 0; i < laserCloudCornerStackNum; i++)
				{
					//转移到世界坐标系
					pointAssociateToMap(&laserCloudCornerStack->points[i], &pointSel);

					//按50的比例尺缩小，四舍五入，偏移laserCloudCen*的量，计算索引
					int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
					int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
					int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

					if (pointSel.x + 25.0 < 0)
						cubeI--;
					if (pointSel.y + 25.0 < 0)
						cubeJ--;
					if (pointSel.z + 25.0 < 0)
						cubeK--;

					if (cubeI >= 0 && cubeI < laserCloudWidth &&
						cubeJ >= 0 && cubeJ < laserCloudHeight &&
						cubeK >= 0 && cubeK < laserCloudDepth)
					{   //只挑选-laserCloudCenWidth * 50.0 < point.x < laserCloudCenWidth * 50.0范围内的点，y和z同理
						//按照尺度放进不同的组，每个组的点数量各异
						int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
						laserCloudCornerArray[cubeInd]->push_back(pointSel);
					}
				}

				//将surf points按距离（比例尺缩小）归入相应的立方体
				for (int i = 0; i < laserCloudSurfStackNum; i++)
				{
					pointAssociateToMap(&laserCloudSurfStack->points[i], &pointSel);

					int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
					int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
					int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

					if (pointSel.x + 25.0 < 0)
						cubeI--;
					if (pointSel.y + 25.0 < 0)
						cubeJ--;
					if (pointSel.z + 25.0 < 0)
						cubeK--;

					if (cubeI >= 0 && cubeI < laserCloudWidth &&
						cubeJ >= 0 && cubeJ < laserCloudHeight &&
						cubeK >= 0 && cubeK < laserCloudDepth)
					{
						int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;
						laserCloudSurfArray[cubeInd]->push_back(pointSel);
					}
				}

				//特征点下采样
				for (int i = 0; i < laserCloudValidNum; i++)
				{
					int ind = laserCloudValidInd[i];

					laserCloudCornerArray2[ind]->clear();
					downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
					downSizeFilterCorner.filter(*laserCloudCornerArray2[ind]); //滤波输出到Array2

					laserCloudSurfArray2[ind]->clear();
					downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
					downSizeFilterSurf.filter(*laserCloudSurfArray2[ind]);

					//Array与Array2交换，即滤波后自我更新
					// no 2???
					pcl::PointCloud<PointType>::Ptr laserCloudTemp = laserCloudCornerArray[ind];
					laserCloudCornerArray[ind] = laserCloudCornerArray2[ind];
					laserCloudCornerArray2[ind] = laserCloudTemp;

					laserCloudTemp = laserCloudSurfArray[ind];
					laserCloudSurfArray[ind] = laserCloudSurfArray2[ind];
					laserCloudSurfArray2[ind] = laserCloudTemp;
				}

				mapFrameCount++;
				//特征点汇总下采样，每隔五帧publish一次，从第一次开始
				if (mapFrameCount >= mapFrameNum)
				{
					mapFrameCount = 0;

					laserCloudSurround->clear();
					for (int i = 0; i < laserCloudSurroundNum; i++)
					{
						int ind = laserCloudSurroundInd[i];
						*laserCloudSurround += *laserCloudCornerArray[ind];
						*laserCloudSurround += *laserCloudSurfArray[ind];
					}

					sensor_msgs::PointCloud2 laserCloudSurround3;
					pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3);
					laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
					laserCloudSurround3.header.frame_id = "/camera_init";
					pubLaserCloudSurround.publish(laserCloudSurround3);
				}

				//将点云中全部点转移到世界坐标系下
				int laserCloudFullResNum = laserCloudFullRes->points.size();
				for (int i = 0; i < laserCloudFullResNum; i++)
				{
					pointAssociateToMap(&laserCloudFullRes->points[i], &laserCloudFullRes->points[i]);
				}

				sensor_msgs::PointCloud2 laserCloudFullRes3;
				pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
				laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				laserCloudFullRes3.header.frame_id = "/camera_init";
				pubLaserCloudFullRes.publish(laserCloudFullRes3);

				odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
				odomAftMapped.pose.pose.orientation.x = q_w_curr.x();
				odomAftMapped.pose.pose.orientation.y = q_w_curr.y();
				odomAftMapped.pose.pose.orientation.z = q_w_curr.z();
				odomAftMapped.pose.pose.orientation.w = q_w_curr.w();
				odomAftMapped.pose.pose.position.x = t_w_curr.x();
				odomAftMapped.pose.pose.position.y = t_w_curr.y();
				odomAftMapped.pose.pose.position.z = t_w_curr.z();
				pubOdomAftMapped.publish(odomAftMapped);

				geometry_msgs::PoseStamped laserAfterMappedPose;
				laserAfterMappedPose.header = odomAftMapped.header;
				laserAfterMappedPose.pose = odomAftMapped.pose.pose;
				laserAfterMappedPath.header.stamp = odomAftMapped.header.stamp;
				laserAfterMappedPath.poses.push_back(laserAfterMappedPose);
				pubLaserAfterMappedPath.publish(laserAfterMappedPath);

				//广播坐标系旋转平移参量
				aftMappedTrans.stamp_ = ros::Time().fromSec(timeLaserOdometry);
				aftMappedTrans.setRotation(tf::Quaternion(q_w_curr.x(), q_w_curr.y(), q_w_curr.z(), q_w_curr.w()));
				aftMappedTrans.setOrigin(tf::Vector3(t_w_curr.x(), t_w_curr.y(), t_w_curr.z()));
				tfBroadcaster.sendTransform(aftMappedTrans);

				printf("mapping pub time %f ms \n", t_pub.toc());
			}
			printf("whole mapping time %f ms +++++\n", t_whole.toc());
		}

		status = ros::ok();
		rate.sleep();
	}

	return 0;
}

// to do list
// x z change
// check 1 init & gt  2 transform to local and world
// plane 0.02