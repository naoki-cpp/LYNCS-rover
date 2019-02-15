#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "../include/Csearch.h"
#define S_MAX 255
#define S_MIN 40
#define V_MAX 255
#define V_MIN 40

using namespace std;
using namespace cv;

int Csearch(int H_MAX1, int H_MIN1, int H_MAX2, int H_MIN2, double coordinate[2])
{
	VideoCapture cap(0);
	cv::Mat inputdata;

	if (!cap.isOpened())
	{
		return -1;
	}

	cap.set(3, 320);
	cap.set(4, 240);

	cap >> inputdata;
	if (inputdata.empty())
	{
		//cout << "end0" << endl;
		return (1);
	}

	//imshow("result",inputdata );
	//imwrite("work1.jpg", inputdata);
	//waitKey(1000);


	Mat mask1, mask2, hsv_image;
	cvtColor(inputdata, hsv_image, COLOR_BGR2HSV, 3);

	Scalar min_edge1 = Scalar(H_MIN1, S_MIN, V_MIN);
	Scalar max_edge1 = Scalar(H_MAX1, S_MAX, V_MAX);
	inRange(hsv_image, min_edge1, max_edge1, mask1);
	Scalar min_edge2 = Scalar(H_MIN2, S_MIN, V_MIN);
	Scalar max_edge2 = Scalar(H_MAX2, S_MAX, V_MAX);
	inRange(hsv_image, min_edge2, max_edge2, mask2);

	Mat mask = mask1 + mask2;
	int Redsum = 0;
	for (int i = 0; i < 320; i++)
	{
		for (int j = 0; j < 240; j++)
		{
			if (mask.data[j * mask.step + i * mask.elemSize() + 0] == 255)
			{
				Redsum++;
			}
		}
	}
	if (Redsum < 1500)
	{
		return (0);
	}
	if (Redsum > 51200)
	{
		return (3);
	}

	Mat element8 = (Mat_<uchar>(3, 3) << 1, 1, 1, 1, 1, 1, 1, 1, 1);
	morphologyEx(mask, mask, MORPH_OPEN, element8, Point(-1, -1), 1);
	morphologyEx(mask, mask, MORPH_CLOSE, element8, Point(-1, -1), 1);
	//imwrite("work2.jpg", mask);
	//imshow("mask", mask);

	//waitKey(5000);

	vector<vector<Point>> contours;
	findContours(mask, contours, RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
	double max_area = 0;
	double area;
	int max_contour = 0;
	if (contours.size() == 0)
	{
		//cout << "none" << endl;
		//cout << "none" << endl;
		return (0);
	}
	for (int i = 0; i < contours.size(); i++)
	{
		area = contourArea(contours.at(i));
		if (max_area < area)
		{
			max_area = area;
			max_contour = i;
		}
	}

	int pointcount = contours.at(max_contour).size();
	coordinate[0] = 0;
	coordinate[1] = 0;
	for (int j = 0; j < pointcount; j++)
	{
		coordinate[0] += contours.at(max_contour).at(j).x;
		coordinate[1] += contours.at(max_contour).at(j).y;
	}
	coordinate[0] = coordinate[0] / pointcount;
	coordinate[1] = coordinate[1] / pointcount;
	//cout << Redsum << endl;

	//circle(inputdata, cv::Point(coordinate[0], coordinate[1]), 30, Scalar(0, 255, 0), 2, 4);
	//imshow("result", inputdata);
	//imwrite("work3.jpg", inputdata);
	//waitKey(5000);
	return (2);
}
