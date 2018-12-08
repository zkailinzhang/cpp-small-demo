#include <iostream>
#include <fstream>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\imgproc\imgproc.hpp>

using namespace cv;
using namespace std;

// 遍历图像像素时，，，(uchar)加或者默认 都是输出acsii码形式，，（int）加就可以了

inline bool check(double const x) {
	return 0 <= x && x <= 1;
}

int main() 
{
	

// 	std::ifstream file("sdsss");
// 	if (!file) {
// 		std::cerr << "无法打开文件\n";
// 		return __LINE__;
// 	}
// 
// 	int num;
// 	double x, y, z;
// 	int cnt = 0;
// 	while (file >> num >> x >> y >> z) {
// 		if (check(x) && check(y) && check(z)) {
// 			++cnt;
// 			std::cout << num << " " << x << " " << y << " " << z << "\n";
// 		}
// 	}
// 	std::cout << "总共" << cnt << "个坐标满足条件\n";

// 	enum
// 	{
// 		/* 8bit, color or not */
// 		CV_LOAD_IMAGE_UNCHANGED  =-1,
// 		/* 8bit, gray */
// 		CV_LOAD_IMAGE_GRAYSCALE  =0,
// 		/* ?, color */
// 		CV_LOAD_IMAGE_COLOR      =1,
// 		/* any depth, ? */
// 		CV_LOAD_IMAGE_ANYDEPTH   =2,
// 		/* ?, any color */
// 		CV_LOAD_IMAGE_ANYCOLOR   =4
// 	};



	Mat img_rgb;
	img_rgb = imread("积木.jpg",1);
	//img_rgb = imread("深度.PNG",CV_LOAD_IMAGE_ANYDEPTH);

	namedWindow("qq",WINDOW_AUTOSIZE);
	imshow("qq",img_rgb);
	waitKey(20);



	for (int time_color=255; time_color >0; time_color--)
	{

	 
	std::stringstream saveName;
	saveName <<"data\\time_color_"<< time_color <<".txt";


	ofstream write2txt( saveName.str());
	
	
 	float dims[10][10][10];
	float x,y,z;
	if (write2txt.is_open()) 
	 {
			for (int i =0; i< 10;i++)
		 	{
				for (int k =0; k< 10;k++)
		 		{
	 				for (int j =0; j< 10;j++)
	 				{
					//dims[i][k][j] = j;
					//dims[i][k][j] = 1024 * rand () / (RAND_MAX + 1.0f);
					write2txt << i<<'\x20'<<k<<'\x20'<<j<< '\n';

					}
				}
			}

			write2txt.close();

	}

	}

// 	if (write2txt.is_open())
// 	{
// 	for(int j=0; j<img_rgb.rows; j++)      
// 	{
// 		for(int i=0; i<img_rgb.cols; i++)                  
// 		{
// 				//cout<<(int)img_rgb.at<cv::Vec3b>(j,i)[0]<<img_rgb.at<cv::Vec3b>(j,i)[1]<<img_rgb.at<cv::Vec3b>(j,i)[2] <<endl;
// 				write2txt <<(int)img_rgb.at<cv::Vec3b>(j,i)[0]<<'\x20'<<(int)img_rgb.at<cv::Vec3b>(j,i)[1]<<'\x20'<<(int)img_rgb.at<cv::Vec3b>(j,i)[2]<< '\n';		
// 
// 		}
// 	}


// 	cv::Mat_<Vec3b>::iterator it = img_rgb.begin<cv::Vec3b>();
// 	cv::Mat_<Vec3b>::iterator itend = img_rgb.end<cv::Vec3b>();
// 
// 	for (; it != itend; it++)
// 	{
// 
// 	
// 		cout<<(uchar)(*it)[0]<<" "<<(int)(*it)[1]<<" "<<(int)(*it)[2]<<endl;
// 
// 
// 	}
















	return 0;
}