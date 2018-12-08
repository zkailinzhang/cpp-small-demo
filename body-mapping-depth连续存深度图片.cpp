// body-mapping-depth.cpp : 定义控制台应用程序的入口点。
//

// CoodinateMapper.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <kinect.h>

using namespace cv;
using namespace std;

template<class Interface>
inline void SafeRelease( Interface *& pInterfaceToRelease )
    {
    if ( pInterfaceToRelease != NULL )
        {
        pInterfaceToRelease->Release();
        pInterfaceToRelease = NULL;
        }
    }

int _tmain(int argc, _TCHAR* argv[])
    {
        //KinectSensor
        IKinectSensor* pSensor ;
        HRESULT hResult = S_OK ;
        hResult = GetDefaultKinectSensor( &pSensor );
        if ( FAILED( hResult ) )
            {
              cerr << "Error:GetDefaultKinectSensor" << endl;
              return -1;
            }

        hResult = pSensor->Open();
        if ( FAILED( hResult ) )
            {
              cerr << "Error:IKinectSensor::Open()" << endl;
              
              
              
              
              
                 
              return -1;
            }

        //Source
        IDepthFrameSource* pDepthSource;
        hResult = pSensor->get_DepthFrameSource( &pDepthSource );      
        if ( FAILED( hResult ) )
            {
              cerr << "Error:IKinectSensor::get_DepthFrameSource()" << endl;
              return -1;
            }

        IBodyIndexFrameSource* pBodyIndexSource;                             // 骨骼跟踪  身体索引 这里做掩膜 提取前景即人
        hResult = pSensor->get_BodyIndexFrameSource( &pBodyIndexSource );
        if (FAILED(hResult))
            {
              cerr << "Error:IKinectSensor::get_BodyIndexFrameSource()" << endl;
              return -1;
            }

        //Reader
        IDepthFrameReader* pDepthReader;
        hResult = pDepthSource->OpenReader( &pDepthReader );           
        if ( FAILED( hResult ) )
            {
              cerr << "Error:IDepthFrameSource::OpenReader()" << endl;
              return -1;
            }

        IBodyIndexFrameReader* pBodyIndexReader;
        hResult = pBodyIndexSource->OpenReader( &pBodyIndexReader );
        if (FAILED(hResult))
            {
              cerr << "Error:IBodyIndexFrameReader::OpenReader()" << endl;
              return -1;
            }  

        //Description
        IFrameDescription* pDepthDescription;
        hResult = pDepthSource->get_FrameDescription( &pDepthDescription );
        if ( FAILED( hResult ) )
            {
              cerr << "Error:IDepthFrameSource::get_FrameDescription()" << endl;
              return -1;
            }

        int depthWidth = NULL;
        int depthHeight = NULL;
        pDepthDescription->get_Width( &depthWidth );
        pDepthDescription->get_Height( &depthHeight );
        unsigned int depthBufferSize = depthWidth * depthHeight * sizeof( unsigned short );




        Mat depthBufferMat( depthHeight, depthWidth, CV_16UC1 );
        Mat depthMat( depthHeight, depthWidth,CV_8UC1 );
        namedWindow( "depth", CV_WINDOW_NORMAL );


        IFrameDescription* pbodyindexDescription;
        hResult = pDepthSource->get_FrameDescription( &pbodyindexDescription );
        if ( FAILED( hResult ) )
            {
            cerr << "Error:IDepthFrameSource::get_FrameDescription()" << endl;
            return -1;
            }

        int bodyindexWidth = NULL;
        int bodyindexHeight = NULL;
        pbodyindexDescription->get_Width( &bodyindexWidth );
        pbodyindexDescription->get_Height( &bodyindexHeight );

        Mat bodyIndexMat( bodyindexHeight, bodyindexWidth, CV_16UC1 );    
 //       Mat bodyIndexMat( bodyindexHeight, bodyindexWidth, CV_8UC1 );
   //     Mat bodyIndexMat( bodyindexHeight, bodyindexWidth, CV_16UC1 );    
        namedWindow("BodyIndex", CV_WINDOW_NORMAL);

        string dst_img_name = "K://数据输出//body-mapping-depth//";
        char chari[10000];
        int n = 0;
        string  dst_xml ;
        string dst_xml_no;

        while (1)
            {
              // Frame
               IDepthFrame* pDepthFrame = nullptr;
               hResult = pDepthReader->AcquireLatestFrame( &pDepthFrame );
               if ( SUCCEEDED(hResult) )
                 {
                hResult = pDepthFrame->AccessUnderlyingBuffer( &depthBufferSize, reinterpret_cast<UINT16**>( &depthBufferMat.data));
                if ( SUCCEEDED(hResult) )
                    {
                    depthBufferMat.convertTo( depthMat, CV_8U, -255.0f/8000.0f, 255.0f );
                    }
                 }

               IBodyIndexFrame* pBodyIndexFrame = nullptr;
               hResult = pBodyIndexReader->AcquireLatestFrame( &pBodyIndexFrame );
               if (SUCCEEDED(hResult))
                  {
                      unsigned int bufferSize = 0;
                      unsigned char* buffer = nullptr;
                      hResult = pBodyIndexFrame->AccessUnderlyingBuffer( &bufferSize, &buffer );       
                        if (SUCCEEDED(hResult))
                           {
                              for ( int y = 0; y < depthHeight; y++ )
                              {
                                 for ( int x = 0; x < depthWidth; x++ )
                                 {
                                    unsigned int index = y * depthWidth + x;
                                    if (buffer[index] != 0xff )
                                    {
                                    //  bodyIndexMat.at<UINT8>( y, x ) = depthMat.at<UINT8>(y,x);
                                      bodyIndexMat.at<UINT16>( y, x ) = depthBufferMat.at<UINT16>(y,x);
                                     }
                                    else
                                    {
                                    //  bodyIndexMat.at<UINT8>( y, x )  = 0;       //**将背景设置成黑色**
                                      bodyIndexMat.at<UINT16>( y, x )  = 0xffff;       //**将背景设置成黑色**
                                     }
                                   }
                                 }
                              }
                      }
                      
                       //对不同形态学操作效果进行比较
                       //定义核  
                  //   Mat element = getStructuringElement(MORPH_RECT, Size(15, 15));   
                      //进行形态学操作  
                  //   morphologyEx( bodyIndexMat, bodyIndexMat, MORPH_OPEN, element);  

                      //filter
                    //中值滤波
                 //   medianBlur( bodyIndexMat, bodyIndexMat, 7);  

                    //双边滤波
                //    bilateralFilter( bodyIndexMat, bodyIndexMat, 25, 25*2, 25/2 );
                //    boxFilter(bodyIndexMat, bodyIndexMat, -1,Size(5, 5));  

                 //   blur(bodyIndexMat, bodyIndexMat, Size(7, 7));  

                 //   GaussianBlur( bodyIndexMat, bodyIndexMat, Size( 5, 5 ), 0, 0 );   

                    imshow( "depth", depthMat );
                    imshow( "BodyIndex", bodyIndexMat );

                    sprintf_s( chari, "%04d", n);

                     dst_img_name += "body-mapping-depth_";
                    dst_img_name += chari;

                    //存储成XML文件
                    dst_xml_no =  "body-mapping-depth_";
                    dst_xml_no += chari;
                    
                    dst_xml = dst_img_name + ".XML";
                    FileStorage fs( dst_xml, FileStorage::WRITE);
                    fs << dst_xml_no << bodyIndexMat;
                    fs.release();

                    dst_img_name += ".PNG";

                    vector<int> compression_params;
                    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
                    compression_params.push_back(9);

                    imwrite( dst_img_name, bodyIndexMat, compression_params);

                    dst_img_name = "K://数据输出//body-mapping-depth//";
                    n++;

                    if ( waitKey( 1 ) == VK_ESCAPE )
                        {
                          break;
                        }   

                   SafeRelease( pDepthFrame );
                   SafeRelease( pBodyIndexFrame );
                 }

            SafeRelease( pDepthReader );
            SafeRelease( pDepthSource );
            SafeRelease( pDepthReader );
            SafeRelease( pDepthDescription );
            if ( pSensor )
                {
                pSensor->Close();
                }
            SafeRelease( pSensor );
            destroyAllWindows();

            return 0;
    }

