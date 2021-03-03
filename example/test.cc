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
#include "ceres/ceres.h"
// #include <ros/log.h>

using namespace ceres;
using namespace std;
struct CostFunctor {
	CostFunctor(double k1, double k2): k1_(k1), k2_(k2)
	{}
	template <typename T>
	bool operator()(const T* const main_k, T* residual) const {
		// residual[0] = T(10.0) - x[0];

		T r1 = T(k1_) - main_k[0] ;
		T r2 = T(k2_) - main_k[0] ;

		if (r1 * r1 < r2 * r2)
		{
			residual[0] = r1;
			T r = T(k2_) - main_k[1];
			residual[1] = r;
		}
		else
		{
			residual[0] = r2;
			T r = T(k1_) - main_k[1];
			residual[1] = r;
		}

		return true;
	}
	double k1_ , k2_;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_detector");
	{
		boost::shared_ptr<sensor_msgs::LaserScan const> scan_ptr;    // 有const
		scan_ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan", ros::Duration(10));
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
			// std::vector<std::vector<cv::Point>> best_pts;
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
					              / ceres::sqrt(A * A + B * B);
					ROS_INFO("line dst:%f", dist);
					// std::cerr << "dst is " << dist << std::endl;
					if (dist * 0.05 <= 0.05)
					{
						// std::cerr << "pop dst is " << dist << std::endl;

						filter_laser_pts_.pop_back();
					}
					else
					{
						// if (dist < 0.789)
						// {
						// 	double pt_dst = sqrt(pow(last_pt.x - pt.x, 2) +
						// 	                     pow(last_pt.y - pt.y, 2));
						// 	if (pt_dst < 0.789)
						// 	{
						// 		filter_laser_pts_.push_back(last_pt);

						// 		filter_laser_pts_.push_back(pt);
						// 	}

						// }
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
			// std::vector<Eigen::Vector3d> line_params;
			for (int i = 0; i < filter_laser_pts_.size() - 2; i = i + 2)
			{
				cv::line(tmp_line, filter_laser_pts_[i], filter_laser_pts_[i + 1], cv::Scalar(180), 2);
				cv::imshow("test", tmp_line);
				cv::waitKey(15);

				double A = filter_laser_pts_[i].y - filter_laser_pts_[i + 1].y;
				double B = filter_laser_pts_[i + 1].x - filter_laser_pts_[i].x;
				double C = filter_laser_pts_[i].x * filter_laser_pts_[i + 1].y
				           - filter_laser_pts_[i + 1].x * filter_laser_pts_[i].y;
				line_params.push_back(Eigen::Vector3d(A, B, C));
			}
			cv::imshow("test", tmp_line);
			cv::waitKey(1);
			double main_k1 = 1;
			double main_k2 = -1;
			double main_k[2] = {1, -1};
			// double main_k[2] = { -line_params.front()[0] / line_params.front()[1], line_params.front()[1] / line_params.front()[0]};

			Problem problem;

			// Set up the only cost function (also known as residual). This uses
			// auto-differentiation to obtain the derivative (jacobian).
			// CostFunction* cost_function =
			//     new AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor);
			// problem.AddResidualBlock(cost_function, NULL, &x);

			// // Run the solver!
			Solver::Options options;
			// options.linear_solver_type = ceres::DENSE_QR;
			options.minimizer_progress_to_stdout = true;
			Solver::Summary summary;
			for (auto&params : line_params)
			{
				if (params[1] == 0)
				{
					params[1] = 0.000001;
				}
				if (params[0] == 0)
				{
					params[0] = 0.000001;
				}
				problem.AddResidualBlock (     // 向问题中添加误差项
				    // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
				    new ceres::AutoDiffCostFunction<CostFunctor, 2, 2> (
				        new CostFunctor ( -params[0] / params[1], params[1] / params[0] )
				    ),
				    new ceres::CauchyLoss(0.5),            // 核函数，这里不使用，为空
				    main_k                 // 待估计参数
				);
			}
			Solve(options, &problem, &summary);
			cout << summary.BriefReport() << endl;
			cout<< "angle is " << main_k[0] << endl;
		}

		// ros::spinOnce();
	}
	cv::waitKey();
	ros::shutdown();
	return 0;
}