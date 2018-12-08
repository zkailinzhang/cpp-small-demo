// img-continus-imread.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>

using namespace cv;
using namespace std;

const int NUM = 1000;

int _tmain(int argc, _TCHAR* argv[])
{
   char filename[1000];
   char dst_img_xml[1000];
   char dst_img[1000];
   Mat img;
   namedWindow( "color", CV_WINDOW_NORMAL );
   
   for ( int i = 89; i < NUM; i++ )
   {
     sprintf_s( filename, "K://数据输出//color//color_%04d.PNG", i );     // "K://数据输出//color//color_%04d.PNG"代表图片所在的位置
     sprintf_s( dst_img_xml, "K://数据输出//color//color_%04d.xml", i );  // "K://数据输出//color//color_%04d.xml" ，代表所要write的地址、图片编号（color_%04d或直接认为（color_i））和类型 （.xml）
     sprintf_s( dst_img, "color_%04d", i);

     img = imread( filename, 1);  
     imshow( "color", img );

     FileStorage fs( dst_img_xml, FileStorage::WRITE );
     fs << dst_img << img;                   //dst_img代表图片的名字、编号，如 color_100

     cout << i <<endl;
     if ( waitKey(30) >=27 )
         {
           break;
         }
   }            
	return 0;
}

