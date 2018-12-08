// img-continus-imread.cpp : �������̨Ӧ�ó������ڵ㡣
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
     sprintf_s( filename, "K://�������//color//color_%04d.PNG", i );     // "K://�������//color//color_%04d.PNG"����ͼƬ���ڵ�λ��
     sprintf_s( dst_img_xml, "K://�������//color//color_%04d.xml", i );  // "K://�������//color//color_%04d.xml" ��������Ҫwrite�ĵ�ַ��ͼƬ��ţ�color_%04d��ֱ����Ϊ��color_i���������� ��.xml��
     sprintf_s( dst_img, "color_%04d", i);

     img = imread( filename, 1);  
     imshow( "color", img );

     FileStorage fs( dst_img_xml, FileStorage::WRITE );
     fs << dst_img << img;                   //dst_img����ͼƬ�����֡���ţ��� color_100

     cout << i <<endl;
     if ( waitKey(30) >=27 )
         {
           break;
         }
   }            
	return 0;
}

