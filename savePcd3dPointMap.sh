#! /bin/sh
#找到前端算法建立map，发布的点云话题
#rosbag record该话题
#

roscore
rosbag play x1.bag
rosrun pcl_ros pointcloud_to_pcd input:=/x2
#rosrun pcl_ros bag_to_pcd xxx.bag /laser_cloud_surround pcd


