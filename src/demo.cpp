#include"../include/MR_Position.h"
int main(int argc, char **argv)
{
	cv::VideoCapture cap(0);
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
	if (cap.isOpened())
	{
		cv::Mat frame;
		MR_POSE pose;
		while (true)
		{
			cap >> frame;
			if (frame.empty()) break;
			cv::String xml_path = "D:/CODEing/OpenCV_codeSources/CameraCalibrate/C930E_320x240/";
			cv::String filename = "C930E_320x240_calibMatrixs.xml";
			pose.getCameraParamiterFromXML(xml_path+filename);
			pose.run_MR_POSE(frame);
			char c = cv::waitKey(1);
			if (c == 27) break;
		}
	}
	return system("pause");
}