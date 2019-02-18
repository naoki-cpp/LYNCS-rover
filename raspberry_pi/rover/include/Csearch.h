#pragma once
namespace cv{
	class VideoCapture;
	class Mat;
}
class Csearch
{
  private:
	cv::VideoCapture* cap_;
	cv::Mat* inputdata_;

  public:
	Csearch(/* args */);
	~Csearch();
	int Init();
	int Search(int H_MAX1, int H_MIN1, int H_MAX2, int H_MIN2, double coordinate[2]);
};