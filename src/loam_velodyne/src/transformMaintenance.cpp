// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

#include <cmath>

#include <loam_velodyne/common.h>
#include <nav_msgs/Odometry.h>
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

//odometry计算的转移矩阵(实时高频量)
float transformSum[6] = {0};
//平移增量
float transformIncre[6] = {0};
//经过mapping矫正过后的最终的世界坐标系下的位姿
float transformMapped[6] = {0};
//mapping传递过来的优化前的位姿
float transformBefMapped[6] = {0};
//mapping传递过来的优化后的位姿
float transformAftMapped[6] = {0};

ros::Publisher *pubLaserOdometry2Pointer = NULL;
tf::TransformBroadcaster *tfBroadcaster2Pointer = NULL;
nav_msgs::Odometry laserOdometry2;
tf::StampedTransform laserOdometryTrans2;

//odometry的运动估计和mapping矫正量融合之后得到的最终的位姿transformMapped
void transformAssociateToMap()
{
  //平移后绕y轴旋转（-transformSum[1]）
  float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
           - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
  float y1 = transformBefMapped[4] - transformSum[4];
  float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
           + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

  //绕x轴旋转（-transformSum[0]）
  float x2 = x1;
  float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
  float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

  //绕z轴旋转（-transformSum[2]）
  transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
  transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
  transformIncre[5] = z2;

  float sbcx = sin(transformSum[0]);
  float cbcx = cos(transformSum[0]);
  float sbcy = sin(transformSum[1]);
  float cbcy = cos(transformSum[1]);
  float sbcz = sin(transformSum[2]);
  float cbcz = cos(transformSum[2]);

  float sblx = sin(transformBefMapped[0]);
  float cblx = cos(transformBefMapped[0]);
  float sbly = sin(transformBefMapped[1]);
  float cbly = cos(transformBefMapped[1]);
  float sblz = sin(transformBefMapped[2]);
  float cblz = cos(transformBefMapped[2]);

  float salx = sin(transformAftMapped[0]);
  float calx = cos(transformAftMapped[0]);
  float saly = sin(transformAftMapped[1]);
  float caly = cos(transformAftMapped[1]);
  float salz = sin(transformAftMapped[2]);
  float calz = cos(transformAftMapped[2]);

  float srx = -sbcx*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz)
            - cbcx*sbcy*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
            - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
            - cbcx*cbcy*(calx*salz*(cblz*sbly - cbly*sblx*sblz) 
            - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx);
  transformMapped[0] = -asin(srx);

  float srycrx = sbcx*(cblx*cblz*(caly*salz - calz*salx*saly)
               - cblx*sblz*(caly*calz + salx*saly*salz) + calx*saly*sblx)
               - cbcx*cbcy*((caly*calz + salx*saly*salz)*(cblz*sbly - cbly*sblx*sblz)
               + (caly*salz - calz*salx*saly)*(sbly*sblz + cbly*cblz*sblx) - calx*cblx*cbly*saly)
               + cbcx*sbcy*((caly*calz + salx*saly*salz)*(cbly*cblz + sblx*sbly*sblz)
               + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) + calx*cblx*saly*sbly);
  float crycrx = sbcx*(cblx*sblz*(calz*saly - caly*salx*salz)
               - cblx*cblz*(saly*salz + caly*calz*salx) + calx*caly*sblx)
               + cbcx*cbcy*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx)
               + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) + calx*caly*cblx*cbly)
               - cbcx*sbcy*((saly*salz + caly*calz*salx)*(cbly*sblz - cblz*sblx*sbly)
               + (calz*saly - caly*salx*salz)*(cbly*cblz + sblx*sbly*sblz) - calx*caly*cblx*sbly);
  transformMapped[1] = atan2(srycrx / cos(transformMapped[0]), 
                             crycrx / cos(transformMapped[0]));
  
  float srzcrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
               - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
               - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
               - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
               + cbcx*sbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
  float crzcrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*calz*(cbly*sblz - cblz*sblx*sbly)
               - calx*salz*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sbly)
               - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*salz*(cblz*sbly - cbly*sblx*sblz)
               - calx*calz*(sbly*sblz + cbly*cblz*sblx) + cblx*cbly*salx)
               + cbcx*cbcz*(salx*sblx + calx*cblx*salz*sblz + calx*calz*cblx*cblz);
  transformMapped[2] = atan2(srzcrx / cos(transformMapped[0]), 
                             crzcrx / cos(transformMapped[0]));

  x1 = cos(transformMapped[2]) * transformIncre[3] - sin(transformMapped[2]) * transformIncre[4];
  y1 = sin(transformMapped[2]) * transformIncre[3] + cos(transformMapped[2]) * transformIncre[4];
  z1 = transformIncre[5];

  x2 = x1;
  y2 = cos(transformMapped[0]) * y1 - sin(transformMapped[0]) * z1;
  z2 = sin(transformMapped[0]) * y1 + cos(transformMapped[0]) * z1;

  transformMapped[3] = transformAftMapped[3] 
                     - (cos(transformMapped[1]) * x2 + sin(transformMapped[1]) * z2);
  transformMapped[4] = transformAftMapped[4] - y2;
  transformMapped[5] = transformAftMapped[5] 
                     - (-sin(transformMapped[1]) * x2 + cos(transformMapped[1]) * z2);
}

//接收laserOdometry的信息
void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  //得到旋转平移矩阵
  transformSum[0] = -pitch;
  transformSum[1] = -yaw;
  transformSum[2] = roll;

  transformSum[3] = laserOdometry->pose.pose.position.x;
  transformSum[4] = laserOdometry->pose.pose.position.y;
  transformSum[5] = laserOdometry->pose.pose.position.z;

  transformAssociateToMap();

  geoQuat = tf::createQuaternionMsgFromRollPitchYaw
            (transformMapped[2], -transformMapped[0], -transformMapped[1]);

  laserOdometry2.header.stamp = laserOdometry->header.stamp;
  laserOdometry2.pose.pose.orientation.x = -geoQuat.y;
  laserOdometry2.pose.pose.orientation.y = -geoQuat.z;
  laserOdometry2.pose.pose.orientation.z = geoQuat.x;
  laserOdometry2.pose.pose.orientation.w = geoQuat.w;
  laserOdometry2.pose.pose.position.x = transformMapped[3];
  laserOdometry2.pose.pose.position.y = transformMapped[4];
  laserOdometry2.pose.pose.position.z = transformMapped[5];
  pubLaserOdometry2Pointer->publish(laserOdometry2);

  //发送旋转平移量
  laserOdometryTrans2.stamp_ = laserOdometry->header.stamp;
  laserOdometryTrans2.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
  laserOdometryTrans2.setOrigin(tf::Vector3(transformMapped[3], transformMapped[4], transformMapped[5]));
  tfBroadcaster2Pointer->sendTransform(laserOdometryTrans2);
}

//接收laserMapping的转换信息
void odomAftMappedHandler(const nav_msgs::Odometry::ConstPtr& odomAftMapped)
{
  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = odomAftMapped->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  transformAftMapped[0] = -pitch;
  transformAftMapped[1] = -yaw;
  transformAftMapped[2] = roll;

  transformAftMapped[3] = odomAftMapped->pose.pose.position.x;
  transformAftMapped[4] = odomAftMapped->pose.pose.position.y;
  transformAftMapped[5] = odomAftMapped->pose.pose.position.z;

  transformBefMapped[0] = odomAftMapped->twist.twist.angular.x;
  transformBefMapped[1] = odomAftMapped->twist.twist.angular.y;
  transformBefMapped[2] = odomAftMapped->twist.twist.angular.z;

  transformBefMapped[3] = odomAftMapped->twist.twist.linear.x;
  transformBefMapped[4] = odomAftMapped->twist.twist.linear.y;
  transformBefMapped[5] = odomAftMapped->twist.twist.linear.z;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "transformMaintenance");
  ros::NodeHandle nh;

  ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry> 
                                     ("/laser_odom_to_init", 5, laserOdometryHandler);

  ros::Subscriber subOdomAftMapped = nh.subscribe<nav_msgs::Odometry> 
                                     ("/aft_mapped_to_init", 5, odomAftMappedHandler);

  ros::Publisher pubLaserOdometry2 = nh.advertise<nav_msgs::Odometry> ("/integrated_to_init", 5);
  pubLaserOdometry2Pointer = &pubLaserOdometry2;
  laserOdometry2.header.frame_id = "/camera_init";
  laserOdometry2.child_frame_id = "/camera";

  tf::TransformBroadcaster tfBroadcaster2;
  tfBroadcaster2Pointer = &tfBroadcaster2;
  laserOdometryTrans2.frame_id_ = "/camera_init";
  laserOdometryTrans2.child_frame_id_ = "/camera";

  ros::spin();

  return 0;
}
