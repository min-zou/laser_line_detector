#include <vector>
#include <string>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <laser_geometry/laser_geometry.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <pcl/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <Eigen/Eigen>
// #include <ros/log.h>
int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_detector");
	{
		boost::shared_ptr<sensor_msgs::LaserScan const> scan_ptr;    // 有const
		scan_ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan", ros::Duration(3));
		tf::TransformListener tfListener_;
		if (scan_ptr != NULL)
		{
			sensor_msgs::PointCloud2 cloud;
			laser_geometry::LaserProjection projector_;
			projector_.transformLaserScanToPointCloud("base_scan", *scan_ptr, cloud, tfListener_);

			int row_step = cloud.row_step;
			int height = cloud.height;

			/*将 sensor_msgs::PointCloud2 转换为　pcl::PointCloud<T> */
			//注意要用fromROSMsg函数需要引入pcl_versions（见头文件定义）
			pcl::PointCloud<pcl::PointXYZ> rawCloud;
			pcl::fromROSMsg(cloud, rawCloud);
			double min_x = 30, min_y = 30, max_x = -1, max_y = -1;
			for (size_t i = 0; i < rawCloud.points.size(); i++) {
				// std::cout << rawCloud.points[i].x << "\t" << rawCloud.points[i].y << "\t" << rawCloud.points[i].z << std::endl;
				double x = rawCloud.points[i].x;
				double y = rawCloud.points[i].y;
				min_y = min_y < y ? min_y : y;
				min_x = min_x < x ? min_x : x;
				max_y = max_y > y ? max_y : y;
				max_x = max_x > x ? max_x : x;
			}
			ROS_INFO("fuch");
			// std::cout << rawCloud.points[0].x << "\t" << rawCloud.points[0].y << "\t" << rawCloud.points[0].z << std::endl;
			double or_x = min_x - 4;
			double or_y = min_y - 4;
			int h = (max_y - min_y + 8) / 0.05 ;
			int width = (max_x - min_x + 8) / 0.05 ;
			cv::Mat tmp_line = cv::Mat::zeros(cv::Size(width, h), CV_8UC1);
			std::vector<cv::Point> laser_pts;
			for (size_t i = 0; i < rawCloud.points.size(); i++) {
				// std::cout << rawCloud.points[i].x << "\t" << rawCloud.points[i].y << "\t" << rawCloud.points[i].z << std::endl;
				double x = rawCloud.points[i].x;
				double y = rawCloud.points[i].y;
				int cell_x = (x - or_x) / 0.05;
				int cell_y =  h -  (y - or_y) / 0.05;
				// tmp_line.at<uchar>(cell_y,cell_x) =

				laser_pts.push_back(cv::Point(cell_x, cell_y));
			}
			std::vector<Eigen::Vector3d> line_params;
			ROS_INFO("line size:%d", laser_pts.size());

			// cv::Point last_pt ;
			int first = 1;
			int count = 0;
			Eigen::Vector3d tmp_line_param;
			std::vector<cv::Point> filter_laser_pts_;
			for (auto&pt : laser_pts)
			{

				if (filter_laser_pts_.size() <= 1 || filter_laser_pts_.size() % 2 != 0)
				{
					filter_laser_pts_.push_back(pt);
				}
				else
				{
					auto& last_pt = filter_laser_pts_.back();
					auto& line_start_pt = filter_laser_pts_[filter_laser_pts_.size() - 2];
					double A = last_pt.y - line_start_pt.y;
					double B = line_start_pt.x - last_pt.x;
					double C = last_pt.x * line_start_pt.y
					           - line_start_pt.x * last_pt.y;
					double dist = fabs(A * pt.x * 1.0 + B * pt.y * 1.0 + C)
					              / sqrt(A * A + B * B);
					ROS_INFO("line dst:%f", dist);
					// std::cerr << "dst is " << dist << std::endl;
					if (dist * 0.05 < 0.08)
					{
						// std::cerr << "pop dst is " << dist << std::endl;

						filter_laser_pts_.pop_back();
					}
					else
					{

						if (dist < 0.789)
						{
							double pt_dst = sqrt(pow(last_pt.x - pt.x, 2) +
							                     pow(last_pt.y - pt.y, 2));
							if (pt_dst < 0.789)
							{
								filter_laser_pts_.push_back(last_pt);

								filter_laser_pts_.push_back(pt);
							}

						}
					}
					filter_laser_pts_.push_back(pt);
				}
			}
			// if (filter_laser_pts_.size() % 2 != 0)
			// {
			// 	filter_laser_pts_.push_back(laser_pts.back());
			// }
			// for (auto&pt : filter_laser_pts_)
			// {
			// 	cv::circle(tmp_line, cv::Point(pt.x, pt.y), 2, cv::Scalar(255, 255, 255), -1);
			// 	// cv::imshow("test", tmp_line);
			// 	// cv::waitKey(5);
			// }
			for (int i = 0; i < filter_laser_pts_.size() - 2; i = i + 2)
			{
				cv::line(tmp_line, filter_laser_pts_[i], filter_laser_pts_[i + 1], cv::Scalar(180), 2);
				cv::imshow("test", tmp_line);
				cv::waitKey(15);
			}
			cv::imshow("test", tmp_line);
			cv::waitKey(1);

		}

		// ros::spinOnce();
	}
	cv::waitKey();
	ros::shutdown();
	return 0;
}