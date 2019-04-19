#ifndef _MR_POSITION_H
#define _MR_POSITION_H
#include <iostream>
#include <chrono>
#include<opencv2/opencv.hpp>
#include<opencv2/features2d.hpp>
#include<opencv2/xfeatures2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

using namespace std;
using namespace g2o;
class MR_POSE
{
public:
	MR_POSE();
	~MR_POSE();
	void getCameraParamiterFromXML(cv::String file)
	{
		cv::FileStorage cameraXML;
		cameraXML.open( file, cv::FileStorage::READ);
		cameraXML["CameraMatrix"] >> cameraMatrix;
		cameraXML["distCoeffs"] >> discoffes;
		cameraXML.release();
	}
	void run_MR_POSE(cv::Mat frame);
	double get_X_distance() { return transport.at<double>(0, 0); }
	double get_Y_distance() { return transport.at<double>(1, 0); }
	double get_Z_distance() { return transport.at<double>(2, 0); }
private:
	cv::Point2d pixel2cam(const cv::Point2d& p, const cv::Mat& K)
	{
		return cv::Point2d
		(
			(p.x - K.at<double>(0, 2)) / K.at<double>(0, 0),
			(p.y - K.at<double>(1, 2)) / K.at<double>(1, 1)
		);
	}
	void bundleAdjustment(
		const vector< cv::Point3f > points_3d,
		const vector< cv::Point2f > points_2d,
		const cv::Mat& K,
		cv::Mat& R, cv::Mat& t);
	cv::Mat cameraMatrix, discoffes;
	cv::Mat rotation, transport;
};
#endif
