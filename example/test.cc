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
#include <csm/csm_all.h>  // csm defines min and max, but Eigen complains

// #include <ros/log.h>
// #include <cmath>
using namespace ceres;
using namespace std;
struct CostFunctor {
	CostFunctor(double k1, double k2): k1_(k1), k2_(k2)
	{}
	template <typename T>
	bool operator()(const T* const main_k, T* residual) const {
		// residual[0] = T(10.0) - x[0];

		T r1 = T(k1_) - main_k[0] ;
		T r2 = T(k1_) + T(1) / main_k[0] ;

		if (r1 * r1 < r2 * r2)
		{
			T r = T(k2_) + T(1) / main_k[0] ;

			residual[0] = r1 * r1 + r * r;
			// residual[1] = r;
		}
		else
		{
			T r = T(k1_) + T(1) / main_k[0] ;

			residual[0] = r2 * r2 + r * r;
			// T r = T(k1_) - main_k[1];
			// residual[1] = r;
		}

		return true;
	}
	double k1_ , k2_;
};
void laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan, LDP& ldp)
{
	unsigned int n = scan->ranges.size();
	ldp = ld_alloc_new(n);

	for (unsigned int i = 0; i < n; i++)
	{
		// Calculate position in laser frame
		double r = scan->ranges[i];
		if ((r > scan->range_min) && (r < scan->range_max))
		{
			// Fill in laser scan data
			ldp->valid[i] = 1;
			ldp->readings[i] = r;
		}
		else
		{
			ldp->valid[i] = 0;
			ldp->readings[i] = -1;  // for invalid range
		}
		ldp->theta[i] = scan->angle_min + i * scan->angle_increment;
		ldp->cluster[i] = -1;
	}
	ROS_WARN("scan size is %d", ldp->cluster[0]);

	ldp->min_theta = ldp->theta[0];
	ldp->max_theta = ldp->theta[n - 1];

	ldp->odometry[0] = 0.0;
	ldp->odometry[1] = 0.0;
	ldp->odometry[2] = 0.0;

	ldp->true_pose[0] = 0.0;
	ldp->true_pose[1] = 0.0;
	ldp->true_pose[2] = 0.0;
}
double get_laser_line(sm_params& input_)
{
	boost::shared_ptr<sensor_msgs::LaserScan const> scan_ptr;    // 有const
	while (scan_ptr == nullptr)
		scan_ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan", ros::Duration(10));

	input_.max_angular_correction_deg = 90;
	input_.restart = 1;
	input_.laser[0] = 0.0;
	input_.laser[1] = 0.0;
	input_.laser[2] = 0.0;
	tf::TransformListener tfListener_;
	double main_k[2] = {1, -1};
	if (scan_ptr != NULL)
	{
		sensor_msgs::PointCloud2 cloud;
		laser_geometry::LaserProjection projector_;
		projector_.transformLaserScanToPointCloud("base_scan", *scan_ptr, cloud, tfListener_);
		// input_.min_reading = scan_ptr->range_min;
		// input_.max_reading = scan_ptr->range_max;
		LDP curr_ldp_scan, prev_ldp_scan_;
		laserScanToLDP(scan_ptr, prev_ldp_scan_);
		// prev_ldp_scan_->odometry[0] = 0.0;
		// prev_ldp_scan_->odometry[1] = 0.0;
		// prev_ldp_scan_->odometry[2] = 0.0;

		prev_ldp_scan_->estimate[0] = 0.0;
		prev_ldp_scan_->estimate[1] = 0.0;
		prev_ldp_scan_->estimate[2] = 0.0;
		input_.first_guess[0] = 0.0;
		input_.first_guess[1] = 0.0;
		// input_.first_guess[2] = 0.0;
		prev_ldp_scan_->true_pose[0] = 0.0;
		prev_ldp_scan_->true_pose[1] = 0.0;
		prev_ldp_scan_->true_pose[2] = 0.0;
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
		// ROS_INFO("line size:%d", laser_pts.size());

		// cv::Point last_pt ;
		int first = 1;
		int count = 0;
		Eigen::Vector3d tmp_line_param;
		std::vector<cv::Point> filter_laser_pts_;
		// std::vector<std::vector<cv::Point>> best_pts;
		cv::Mat show = tmp_line.clone();
		for (auto&pt : laser_pts)
		{
			cv::circle(show, cv::Point(pt.x, pt.y), 2, cv::Scalar(255, 255, 255), -1);

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
				// ROS_INFO("line dst:%f", dist);
				// std::cerr << "dst is " << dist * 0.05 << std::endl;
				if (dist * 0.05 <= 0.1)
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
			cv::imshow("test", show);
			cv::waitKey(1);
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
			// cv::line(tmp_line, filter_laser_pts_[i], filter_laser_pts_[i + 1], cv::Scalar(180), 2);
			// cv::imshow("test", tmp_line);
			// cv::waitKey(15);
			double A = filter_laser_pts_[i].y - filter_laser_pts_[i + 1].y;
			double B = filter_laser_pts_[i + 1].x - filter_laser_pts_[i].x;
			double C = filter_laser_pts_[i].x * filter_laser_pts_[i + 1].y
			           - filter_laser_pts_[i + 1].x * filter_laser_pts_[i].y;
			line_params.push_back(Eigen::Vector3d(A, B, C));
		}
		std::vector<std::vector<cv::Point> > final_lines;
		std::vector<Eigen::Vector3d> final_params;
		for (auto&line : line_params)
		{
			std::vector<cv::Point> pt_line;
			double A = line[0];
			double B = line[1];
			double C = line[2];
			auto pt = laser_pts.begin();
			while (pt != laser_pts.end())
			{
				double dist = fabs(A * pt->x * 1.0 + B * pt->y * 1.0 + C)
				              / ceres::sqrt(A * A + B * B);
				if (dist * 0.05 < 0.15)
				{
					pt_line.push_back(*pt);
					pt = laser_pts.erase(pt);
				}
				else
					pt++;
			}
			if (pt_line.size() > 30)
			{
				final_lines.push_back(pt_line);
				final_params.push_back(line);
			}
		}
		cerr << "finale line size is " << final_lines.size() << endl;

		for (auto&pts : final_lines)
		{
			for (auto&pt : pts)
			{
				cv::circle(tmp_line, cv::Point(pt.x, pt.y), 2, cv::Scalar(255, 255, 255), -1);
			}

			// cv::imshow("test", tmp_line);
			// cv::waitKey(1);
		}


		for (auto&line : final_params)
		{

		}

		// cv::imshow("test", tmp_line);
		// cv::waitKey(1);
		double main_k1 = 1;
		double main_k2 = -1;

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
		options.minimizer_progress_to_stdout = false;
		Solver::Summary summary;
		for (auto&params : final_params)
		{
			if (params[1] == 0)
			{
				params[1] = 0.000001;
			}
			if (params[0] == 0)
			{
				params[0] = 0.000001;
			}
			cout << "origin angle1 is " << -params[0] / params[1] << endl;
			cout << "origin angle2 is " << params[1] / params[0] << endl;
			problem.AddResidualBlock (     // 向问题中添加误差项
			    // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
			    new ceres::AutoDiffCostFunction<CostFunctor, 1, 2> (
			        new CostFunctor ( -params[0] / params[1], params[1] / params[0] )
			    ),
			    new ceres::CauchyLoss(0.5),            // 核函数，这里不使用，为空
			    main_k                 // 待估计参数
			);
		}
		Solve(options, &problem, &summary);
		// cout << summary.BriefReport() << endl;
		cout << "angle1 is " << main_k[0] << endl;
		cout << "angle2 is " << main_k[1] << endl;
		input_.laser_ref = prev_ldp_scan_;

	}

	double line1 = ceres::atan(main_k[0]);
	double line2 =  ceres::atan(-1 / main_k[0]);
	double angle;
	if (fabs(line1) < fabs(line2))
	{
		angle = line1;
	}
	else
	{
		angle = line2;
	}
	return angle;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laser_detector");


	ros::NodeHandle nh;
	sm_params input_;

	{
		if (!nh.getParam ("max_angular_correction_deg", input_.max_angular_correction_deg))
			input_.max_angular_correction_deg = 100.0;

		// Maximum translation between scans (m)
		if (!nh.getParam ("max_linear_correction", input_.max_linear_correction))
			input_.max_linear_correction = 0.50;

		// Maximum ICP cycle iterations
		if (!nh.getParam ("max_iterations", input_.max_iterations))
			input_.max_iterations = 10;

		// A threshold for stopping (m)
		if (!nh.getParam ("epsilon_xy", input_.epsilon_xy))
			input_.epsilon_xy = 0.000001;

		// A threshold for stopping (rad)
		if (!nh.getParam ("epsilon_theta", input_.epsilon_theta))
			input_.epsilon_theta = 0.000001;

		// Maximum distance for a correspondence to be valid
		if (!nh.getParam ("max_correspondence_dist", input_.max_correspondence_dist))
			input_.max_correspondence_dist = 0.3;

		// Noise in the scan (m)
		if (!nh.getParam ("sigma", input_.sigma))
			input_.sigma = 0.010;

		// Use smart tricks for finding correspondences.
		if (!nh.getParam ("use_corr_tricks", input_.use_corr_tricks))
			input_.use_corr_tricks = 1;

		// Restart: Restart if error is over threshold
		if (!nh.getParam ("restart", input_.restart))
			input_.restart = 0;

		// Restart: Threshold for restarting
		if (!nh.getParam ("restart_threshold_mean_error", input_.restart_threshold_mean_error))
			input_.restart_threshold_mean_error = 0.01;

		// Restart: displacement for restarting. (m)
		if (!nh.getParam ("restart_dt", input_.restart_dt))
			input_.restart_dt = 1.0;

		// Restart: displacement for restarting. (rad)
		if (!nh.getParam ("restart_dtheta", input_.restart_dtheta))
			input_.restart_dtheta = 0.1;

		// Max distance for staying in the same clustering
		if (!nh.getParam ("clustering_threshold", input_.clustering_threshold))
			input_.clustering_threshold = 0.25;

		// Number of neighbour rays used to estimate the orientation
		if (!nh.getParam ("orientation_neighbourhood", input_.orientation_neighbourhood))
			input_.orientation_neighbourhood = 20;

		// If 0, it's vanilla ICP
		if (!nh.getParam ("use_point_to_line_distance", input_.use_point_to_line_distance))
			input_.use_point_to_line_distance = 1;

		// Discard correspondences based on the angles
		if (!nh.getParam ("do_alpha_test", input_.do_alpha_test))
			input_.do_alpha_test = 0;

		// Discard correspondences based on the angles - threshold angle, in degrees
		if (!nh.getParam ("do_alpha_test_thresholdDeg", input_.do_alpha_test_thresholdDeg))
			input_.do_alpha_test_thresholdDeg = 20.0;

		// Percentage of correspondences to consider: if 0.9,
		// always discard the top 10% of correspondences with more error
		if (!nh.getParam ("outliers_maxPerc", input_.outliers_maxPerc))
			input_.outliers_maxPerc = 0.90;

		// Parameters describing a simple adaptive algorithm for discarding.
		//  1) Order the errors.
		//  2) Choose the percentile according to outliers_adaptive_order.
		//     (if it is 0.7, get the 70% percentile)
		//  3) Define an adaptive threshold multiplying outliers_adaptive_mult
		//     with the value of the error at the chosen percentile.
		//  4) Discard correspondences over the threshold.
		//  This is useful to be conservative; yet remove the biggest errors.
		if (!nh.getParam ("outliers_adaptive_order", input_.outliers_adaptive_order))
			input_.outliers_adaptive_order = 0.7;

		if (!nh.getParam ("outliers_adaptive_mult", input_.outliers_adaptive_mult))
			input_.outliers_adaptive_mult = 2.0;

		// If you already have a guess of the solution, you can compute the polar angle
		// of the points of one scan in the new position. If the polar angle is not a monotone
		// function of the readings index, it means that the surface is not visible in the
		// next position. If it is not visible, then we don't use it for matching.
		if (!nh.getParam ("do_visibility_test", input_.do_visibility_test))
			input_.do_visibility_test = 0;

		// no two points in laser_sens can have the same corr.
		if (!nh.getParam ("outliers_remove_doubles", input_.outliers_remove_doubles))
			input_.outliers_remove_doubles = 1;

		// If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
		if (!nh.getParam ("do_compute_covariance", input_.do_compute_covariance))
			input_.do_compute_covariance = 0;

		// Checks that find_correspondences_tricks gives the right answer
		if (!nh.getParam ("debug_verify_tricks", input_.debug_verify_tricks))
			input_.debug_verify_tricks = 0;

		// If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
		// incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
		if (!nh.getParam ("use_ml_weights", input_.use_ml_weights))
			input_.use_ml_weights = 0;

		// If 1, the field 'readings_sigma' in the second scan is used to weight the
		// correspondence by 1/sigma^2
		if (!nh.getParam ("use_sigma_weights", input_.use_sigma_weights))
			input_.use_sigma_weights = 0;
	}
	ros::Publisher cmd_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);

	{
		LDP curr_ldp_scan;
		double angle = get_laser_line(input_);
		cerr << "tmp value is " << angle << endl;
		// double line1 = ceres::atan(main_k);
		// double line2 =  ceres::atan(-1 / main_k);
		// double angle;
		// if (fabs(line1) < fabs(line2))
		// {
		// 	angle = line1;
		// }
		// else
		// {
		// 	angle = line2;
		// }
		// double angle = (ceres::atan( -final_params.back()[0] / final_params.back()[1]));
		cerr << 180 * angle / M_PI << endl;
		geometry_msgs::Twist tmp ;
		if (angle > 0)
		{
			tmp.angular.z = -0.03;
			angle *= -1;
		}
		else
		{
			tmp.angular.z = 0.03;

			angle *= -1;
		}

		input_.first_guess[2] = angle;
		double t = fabs(angle) / 0.03 ;
		double t0 = (ros::Time::now()).toSec();
		cerr << t << endl;

		sm_result output_;
		output_.cov_x_m = 0;
		output_.dx_dy1_m = 0;
		output_.dx_dy2_m = 0;
		while (true)
		{
			// double tt = (ros::Time::now()).toSec();
			double t1 = (ros::Time::now()).toSec();
			cmd_pub_.publish(tmp);
			if (t1 - t0 > t)
			{

				cerr << t1 - t0 << endl;
				tmp.angular.z = 0;
				cmd_pub_.publish(tmp);

				boost::shared_ptr<sensor_msgs::LaserScan const> new_scan_ptr = nullptr;  // 有const
				cerr << "restart" << endl;
				while (new_scan_ptr == nullptr) {
					cerr << "wait" << endl;
					new_scan_ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan", ros::Duration(10));
				}

				laserScanToLDP(new_scan_ptr, curr_ldp_scan);
				input_.laser_sens = curr_ldp_scan;
				cerr << "bug-1" << endl;

				if (output_.cov_x_m)
				{
					gsl_matrix_free(output_.cov_x_m);
					output_.cov_x_m = 0;
				}
				if (output_.dx_dy1_m)
				{
					gsl_matrix_free(output_.dx_dy1_m);
					output_.dx_dy1_m = 0;
				}
				if (output_.dx_dy2_m)
				{
					gsl_matrix_free(output_.dx_dy2_m);
					output_.dx_dy2_m = 0;
				}
				cerr << "bug-21" << endl;
				sm_icp(&input_, &output_);
				cerr << "bug-22" << endl;
				double match_angle;

				if (output_.valid)
				{
					// the correction of the laser's position, in the laser frame
					match_angle = output_.x[2];
					input_.first_guess[2] = match_angle;

					if (fabs(match_angle - angle) <= 0.01)
					{
						break;
					}
					else
					{
						if (angle > 0)
						{
							double delta_angle = angle - match_angle ;
							t = fabs(delta_angle) / 0.03;
							t0 = (ros::Time::now()).toSec();
							tmp.angular.z = delta_angle < 0 ? -0.03 : 0.03;
						}
						else
						{
							double delta_angle = angle - match_angle ;
							t = fabs(delta_angle) / 0.03;
							t0 = (ros::Time::now()).toSec();
							tmp.angular.z = delta_angle < 0 ? 0.03 : -0.03;
						}
					}
				}
				else
				{
					ROS_WARN("Error in scan matching");
					cerr << "none result" << endl;
					break;
				}

			}
			// double tt1 = (ros::Time::now()).toSec();
			// cerr <<"time is " << tt1 - tt << endl;
			// break;
		}

		// cmd_pub_.publish(tmp);
		cerr << "over" << endl;

	}

	// ros::spinOnce();

	cv::waitKey(1);
	ros::shutdown();
	return 0;
}