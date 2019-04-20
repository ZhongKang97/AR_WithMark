#include"../include/MR_Position.h"
void MR_POSE::run_MR_POSE(cv::Mat frame)
{
	double t0 = static_cast<double>(cv::getTickCount());//初始时间
	imshow("srcImage", frame);
	cv::Mat frame_gray;
	cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
	//Mat mask=Mat::zeros(frame.size(),CV_8UC1);
	//inRange(frame_gray, 0, 60, mask);
	//imshow("Bimage", mask);
	//Mat frame_masked;
	cv::GaussianBlur(frame_gray, frame_gray, cv::Size(9, 9), 2, 2);
	imshow("gray", frame_gray);
	//frame_gray = 255 - frame_gray;
	//frame_gray.copyTo(frame_masked, mask);
	//imshow("with mask", frame_masked);
	std::vector<cv::Vec3f> circles;
	cv::HoughCircles(frame_gray, circles, cv::HOUGH_GRADIENT, 1.5, 20, 40, 50, 10, 100);
	std::cout << "Circles size " << circles.size() << endl;
	bool find_target = false;
	if (circles.size() == 4)
	{
		double meanRDiff = (std::abs(circles[0][2] - circles[1][2]) + std::abs(circles[0][2] - circles[2][2])
			+ std::abs(circles[0][2] - circles[3][2])) / 3.0;
		if (meanRDiff < 20)
		{
			cv::Point left_up, left_down;
			cv::Point right_up, right_down;
			for (size_t i = 0; i < circles.size(); i++)
			{
				cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
				int radius = cvRound(circles[i][2]);
				//绘制圆心
				circle(frame, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
				//绘制圆轮廓
				circle(frame, center, radius, cv::Scalar(155, 50, 255), 3, 8, 0);
			}
			std::vector<cv::Point2f> left, right;

			int index[4] = { 0,1,2,3 };
			for (int j = 0; j<3; j++)
				for (int i = 0; i < 3 - j; i++)
				{
					if (circles[index[i]][0] > circles[index[i + 1]][0])
					{
						int temp = index[i];
						index[i] = index[i + 1];
						index[i + 1] = temp;
					}
				}

			left.push_back(cv::Point(circles[index[0]][0], circles[index[0]][1]));
			left.push_back(cv::Point(circles[index[1]][0], circles[index[1]][1]));
			right.push_back(cv::Point(circles[index[2]][0], circles[index[2]][1]));
			right.push_back(cv::Point(circles[index[3]][0], circles[index[3]][1]));
			if (left[0].y > left[1].y)
			{
				left_up = left[1];
				left_down = left[0];
			}
			else
			{
				left_up = left[0];
				left_down = left[1];
			}
			if (right[0].y > right[1].y)
			{
				right_up = right[1];
				right_down = right[0];
			}
			else
			{
				right_up = right[0];
				right_down = right[1];
			}
			double length_up, length_down;
			length_up = std::sqrt((right_up.x - left_up.x)*(right_up.x - left_up.x) + (right_up.y - left_up.y)*(right_up.y - left_up.y));
			length_down = std::sqrt((right_down.x - left_down.x)*(right_down.x - left_down.x) + (right_down.y - left_down.y)*(right_down.y - left_down.y));
			if (std::abs(length_down - length_up) < 10)
			{
				find_target = true;
				//cout << "left_up " << left_up << " ,left_down: " << left_down << endl;
				//cout << "right_up " << right_up << " ,right_down: " << right_down << endl;
				line(frame, left_up, right_up, cv::Scalar(0, 0, 255), 2);
				line(frame, right_up, left_down, cv::Scalar(0, 255, 0), 2);
				line(frame, left_down, right_down, cv::Scalar(255, 0, 0), 2);
				std::vector<cv::Point3f> points_3d;
				points_3d.push_back(cv::Point3f(-37.5, -35, 0));
				points_3d.push_back(cv::Point3f(37.5, -35, 0));
				points_3d.push_back(cv::Point3f(-37.5, 35, 0));
				points_3d.push_back(cv::Point3f(37.5, 35, 0));
				std::vector<cv::Point2f> points_2d;
				points_2d.push_back(left_up);
				points_2d.push_back(right_up);
				points_2d.push_back(left_down);
				points_2d.push_back(right_down);
				//cout << "cameramatrix: " << cameraMatrix << endl;
				//cout << "dis: " << discoffes << endl;
				solvePnP(points_3d, points_2d, cameraMatrix, discoffes, rotation, transport, false, cv::SOLVEPNP_P3P);
				cv::Mat r_matrix;
				cv::Rodrigues(rotation, r_matrix);
				bundleAdjustment(points_3d, points_2d, cameraMatrix, r_matrix, transport);
				cv::Rodrigues(r_matrix, rotation);
				//cout << "R_vec = " << rotation*90 << endl;
				//cout << "T_vec=" << transport << endl;
				std::vector<cv::Point3f> points_3d_obj;
				points_3d_obj.push_back(cv::Point3f(0, 0, -40));
				points_3d_obj.push_back(cv::Point3f(0, 40, 0));
				points_3d_obj.push_back(cv::Point3f(40, 0, 0));
				points_3d_obj.push_back(cv::Point3f(0, 0, 0));
				std::vector<cv::Point2f> points_2d_obj;
				projectPoints(points_3d_obj, rotation, transport, cameraMatrix, discoffes, points_2d_obj);
				circle(frame, cv::Point2f(frame.cols / 2, frame.rows / 2), 10, cv::Scalar(0, 0, 255), 2);
				circle(frame, points_2d_obj[3], 10, cv::Scalar(255, 0, 0), 2);
				line(frame, points_2d_obj[3], cv::Point2f(frame.cols / 2, frame.rows / 2), cv::Scalar(0, 0, 255), 3);
				line(frame, points_2d_obj[3], points_2d_obj[0], cv::Scalar(255, 0, 0), 2);
				line(frame, points_2d_obj[3], points_2d_obj[1], cv::Scalar(0, 225, 45), 2);
				line(frame, points_2d_obj[3], points_2d_obj[2], cv::Scalar(0, 25, 255), 2);
				int font_face = cv::FONT_HERSHEY_COMPLEX;//字体 
				double font_scale = 1;//字体大小
				int thickness = 2;//线宽
				std::string text_d_x = cv::format("Distant_X: %f ", transport.at<double>(0, 0));
				std::string text_d_y = cv::format("Distant_Y: %f ", transport.at<double>(1, 0));
				std::string text_d_z = cv::format("Distant_Z: %f ", transport.at<double>(2, 0));
				putText(frame, text_d_x, cv::Point(0, 30), font_face, font_scale, cv::Scalar(255, 0, 135), thickness, 8, 0);
				putText(frame, text_d_y, cv::Point(0, 60), font_face, font_scale, cv::Scalar(255, 0, 135), thickness, 8, 0);
				putText(frame, text_d_z, cv::Point(0, 90), font_face, font_scale, cv::Scalar(255, 0, 135), thickness, 8, 0);
			}
		}
	}
	imshow("circles", frame);
	if (find_target)
	{
		distance_x= transport.at<double>(0, 0);
		distance_y= transport.at<double>(1, 0);
		distance_z= transport.at<double>(2, 0);
	}
	else
	{
		distance_x = 777;
		distance_y = 777;
		distance_z = 777;
	}
	double runtime = (static_cast<double>(cv::getTickCount()) - t0) / cv::getTickFrequency();
	std::cout << "run time perFrame is : " << runtime * 1000 << " ms" << endl;
}


void MR_POSE::bundleAdjustment(
const vector< cv::Point3f > points_3d,
const vector< cv::Point2f > points_2d,
const cv::Mat& K,
cv::Mat& R, cv::Mat& t)
{
// 初始化g2o
typedef g2o::BlockSolver< g2o::BlockSolverTraits<6, 3> > Block;  // pose 维度为 6, landmark 维度为 3
std::unique_ptr<Block::LinearSolverType> linearSolver (new g2o::LinearSolverCSparse<Block::PoseMatrixType>()); // 线性方程求解器
std::unique_ptr<Block> solver_ptr(new Block( std::move(linearSolver) ));     // 矩阵块求解器
g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr)) ;
g2o::SparseOptimizer optimizer;
optimizer.setAlgorithm(solver);
// vertex
g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap(); // camera pose
Eigen::Matrix3d R_mat;
R_mat <<
R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
pose->setId(0);
pose->setEstimate(g2o::SE3Quat(
R_mat,
Eigen::Vector3d(t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0))
));
optimizer.addVertex(pose);

int index = 1;
for (const cv::Point3f p : points_3d)   // landmarks
{
g2o::VertexSBAPointXYZ* point = new g2o::VertexSBAPointXYZ();
point->setId(index++);
point->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
point->setMarginalized(true); // g2o 中必须设置 marg 
optimizer.addVertex(point);
}

// parameter: camera intrinsics
g2o::CameraParameters* camera = new g2o::CameraParameters(
K.at<double>(0, 0), Eigen::Vector2d(K.at<double>(0, 2), K.at<double>(1, 2)), 0
);
camera->setId(0);
optimizer.addParameter(camera);

// edges
index = 1;
for (const cv::Point2f p : points_2d)
{
g2o::EdgeProjectXYZ2UV* edge = new g2o::EdgeProjectXYZ2UV();
edge->setId(index);
edge->setVertex(0, dynamic_cast<g2o::VertexSBAPointXYZ*> (optimizer.vertex(index)));
edge->setVertex(1, pose);
edge->setMeasurement(Eigen::Vector2d(p.x, p.y));
edge->setParameterId(0, 0);
edge->setInformation(Eigen::Matrix2d::Identity());
optimizer.addEdge(edge);
index++;
}

chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
optimizer.setVerbose(false);
optimizer.initializeOptimization();
optimizer.optimize(20);
chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>> (t2 - t1);
std::cout << "optimization costs time: " << time_used.count() << " seconds." << endl;

//cout << endl << "after optimization:" << endl;
//cout << "T=" << endl << Eigen::Isometry3d(pose->estimate()).matrix() << endl;
}
MR_POSE::MR_POSE()
{
}

MR_POSE::~MR_POSE()
{
}