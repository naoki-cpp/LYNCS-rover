#include </usr/local/include/opencv2/opencv.hpp>

#include </usr/local/include/opencv2/core.hpp>
#include </usr/local/include/opencv2/imgproc.hpp>
#include </usr/local/include/opencv2/highgui.hpp>
#define H_MAX1 10
#define H_MIN1 0
#define H_MAX2 180
#define H_MIN2 140
#define S_MAX 255
#define S_MIN 40
#define V_MAX 255
#define V_MIN 100

using namespace std;
using namespace cv;

int main(void){
  VideoCapture cap(0);
  if(!cap.isOpened())  // 成功したかどうかをチェック
          return -1;
  cap.set(3, 320);
  cap.set(4, 240);


   //Mat inputdata = imread("image1.jpg", CV_LOAD_IMAGE_COLOR);
   Mat inputdata;

   cap >> inputdata;
   if (inputdata.empty()){
   cout << "end0" << endl;
     return(1);
   }

   Mat mask1,mask2,hsv_image,picture, gray_picture, gmask,add1,add;
   cvtColor(inputdata, hsv_image, CV_BGR2HSV, 3);
   cvtColor(inputdata,gray_picture, CV_BGR2GRAY,3);
   imshow("main", inputdata);
   waitKey(3000);

   int a;
   int b;
   Mat mask;
   while(1){
     cin >> a;
     cin >> b;
     if(a==1000)break;


   Scalar min_edge1 = Scalar(H_MIN1, S_MIN, V_MIN);
   Scalar max_edge1 = Scalar(a, S_MAX, V_MAX);
   inRange(hsv_image, min_edge1, max_edge1, mask1);
   Scalar min_edge2 = Scalar(b, S_MIN, V_MIN);
   Scalar max_edge2 = Scalar(H_MAX2, S_MAX, V_MAX);
   inRange(hsv_image, min_edge2, max_edge2, mask2);

   mask = mask1 + mask2;
   imshow("element1", mask);
   waitKey(1000);
}
Mat element8 = (Mat_<uchar>(3,3) << 1,1,1,1,1,1,1,1,1);
dilate(mask, mask, element8, Point(-1,-1), 2);
  /* Mat element8 = (Mat_<uchar>(3,3) << 1,1,1,1,1,1,1,1,1);
   morphologyEx(mask, mask, CV_MOP_OPEN, element8, Point(-1,-1), 1);
   morphologyEx(mask, mask, CV_MOP_CLOSE, element8, Point(-1,-1), 1);
   imwrite("work2.jpg", mask);
   imshow("mask", mask);
   */
 //Mat kernel = (Mat_<uchar>(3,3) << 1,1,1,1,-8,1,1,1,1);
 unsigned char lap[3][3] = {{1,1,1},{1,-8,1},{1,1,1}};
 Mat filter(Size(3,3), CV_8S,lap);
 filter2D(gray_picture, gray_picture, gray_picture.depth(), filter);
 //filter2D(gray_picture, gray_picture,-1,kernel,Point(-1,-1),0,BORDER_DEFAULT);
 Scalar min_edge3 = Scalar(20);
 Scalar max_edge3 = Scalar(255);
 inRange(gray_picture, min_edge3, max_edge3, gmask);
 imshow("element2", gmask);
  waitKey(1000);

bitwise_and(mask, gmask, add);
imshow("add", add);
   waitKey(1000);


while(1){
  cin >> a;
  if(a==100)break;
  //morphologyEx(gmask, gmask1, CV_MOP_OPEN, element8, Point(-1,-1), a);
  morphologyEx(add, add1, CV_MOP_CLOSE, element8, Point(-1,-1), a);
  //dilate(gmask, gmask1, element8, Point(-1,-1), a);
  imshow("add", add1);
   waitKey(1000);
}
   /*vector<vector<Point> > contours;
   findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
   double max_area =0;
   double area;
   int max_contour = 0;
   for (int i=0; i < contours.size(); i++){
     area = contourArea(contours.at(i));
     if(max_area< area){
       max_area = area;
       max_contour = i;
     }
   }


   int pointcount =contours.at(max_contour).size();
   double x = 0;
   double y = 0;
   for (int j= 0; j< pointcount; j++){
     x += contours.at(max_contour).at(j).x;
     y += contours.at(max_contour).at(j).y;
   }
   x = x/pointcount;
   y = y/pointcount;
   cout << x << endl;
   cout << y << endl;

    circle(inputdata, cv::Point(x,y), 60, Scalar(0,255,0), 3, 4);
    imshow("result", inputdata);
    imwrite("work3.jpg", inputdata);*/
    return(1);

}
