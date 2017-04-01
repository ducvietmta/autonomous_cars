//get image from astra and show
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/photo/photo.hpp"
#include "opencv2/core/core.hpp"
#include "opencv/cvaux.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <stdio.h>
#include <OpenNI.h>
//#include "api_uart.h" 

using namespace std;
using namespace cv;
using namespace openni;


#define SAMPLE_READ_WAIT_TIMEOUT 2000 //2000ms

vector<int> ansX, ansY;
//int cport_nr;
//char buf_send[BUFF_SIZE];
//char buf_off[BUFF_SIZE];
void DetectLanewithHought(vector<cv::Vec4i> lines, Mat& colorImg, Mat& result)
{
	std::vector<std::vector<std::vector<cv::Point> > > lineSegmentsClusters;

	vector<cv::Point> aux;
	vector<vector<cv::Point> > lineSegments;

	for (size_t i = 0; i<lines.size(); i++)
	{
		cv::Point pt1, pt2;
		pt1.x = lines[i][0];
		pt1.y = lines[i][1];
		pt2.x = lines[i][2];
		pt2.y = lines[i][3];

		double X = (double)(pt2.x - pt1.x);
		double Y = (double)(pt2.y - pt1.y);

		float  angle = atan2(Y, X) * 180 / CV_PI;
		float delta = abs(angle);
		int px = (pt1.x + pt2.x) / 2;		//// X-Center of line
		int py = (pt1.y + pt2.y) / 2;		//// -Center of line
		if ((px>colorImg.cols / 2) && Y>0 && angle>20 && angle<85)
			//if( (px>imgGrayOrigin.cols/2) && Y>0 && angle>25 && angle<85)
		{
			aux.clear();
			aux.push_back(pt1);
			aux.push_back(pt2);
			lineSegments.push_back(aux);
		}
		if ((px<colorImg.cols / 2) && Y<0 && angle>-85 && angle<-20)
		{
			aux.clear();
			aux.push_back(pt1);
			aux.push_back(pt2);
			lineSegments.push_back(aux);
		}
	}
	for (int i = 0; i < lineSegments.size(); i++)
	{
		vector<cv::Point> line = lineSegments.at(i);
		cv::Point pt1 = line.at(0);
		cv::Point pt2 = line.at(1);
		//if (pt1.y > pt2.y + 10 || pt1.y < pt2.y - 10)
		//{	
			//cv::circle(result, Point(px, py), 2, Scalar(255, 0, 0), 5);
			cv::line(result, pt1, pt2, Scalar(255, 0, 0, 255), 2);
		//}
	}
	
}

void analyzeFrame(const VideoFrameRef& frame_depth,VideoFrameRef& frame_color,Mat& depth_img, Mat& color_img)
{
    DepthPixel* depth_img_data;
    RGB888Pixel* color_img_data;

    int w = frame_depth.getWidth();
    int h = frame_depth.getHeight();
    int pointIndex = ((w*140)+450);
    depth_img = Mat(h, w, CV_16U);
    color_img = Mat(h, w, CV_8UC3);
    Mat depth_img_8u;
        depth_img_data = (DepthPixel*)frame_depth.getData();

        memcpy(depth_img.data, depth_img_data, h*w*sizeof(DepthPixel));

        printf("[%08llu] %8d\n", (long long)frame_depth.getTimestamp(),
			depth_img_data[pointIndex]);
        color_img_data = (RGB888Pixel*)frame_color.getData();

        memcpy(color_img.data, color_img_data, h*w*sizeof(RGB888Pixel));

        cvtColor(color_img, color_img, COLOR_RGB2BGR);
        circle(color_img, Point(450, 140), 2, Scalar(0, 255, 0), 3);
        imshow("RGB ", color_img);
}


int main()
{
////////////////////////////////////////////////////////////////////////////////////////
    int left = 0, right = 0;
	//cport_nr = api_uart_open();

	//if (cport_nr == -1) {
	//	cerr << "Error: Canot Open ComPort";
	//	return -1;
	//}
	std::vector<Mat> vps;
	Mat imgBin, imgGray, imgGauss, imgCanny, denoise, abs_grad, contours;
	int erosion_size = 4;
	cv::Mat element = cv::getStructuringElement(MORPH_RECT,                            //
		Size(2 * erosion_size + 1, 2 * erosion_size + 1),                              //
		Point(erosion_size, erosion_size));                                            //
	CvMemStorage storage = CvMemStorage();                                             //
	vector<cv::Vec4i> lines;                                                           //
	vector<vector<cv::Point> > lineSegments; 
                                       //
////////////////////////////////////////////////////////////////////////////////////////	
    Mat depthImg, colorImg;
    Status rc = OpenNI::initialize();
    if (rc != STATUS_OK)
    {
        printf("Initialize failed\n%s\n", OpenNI::getExtendedError());
        return 1;
    }

    Device device;
    rc = device.open(ANY_DEVICE);
    if (rc != STATUS_OK)
    {
        printf("Couldn't open device\n%s\n", OpenNI::getExtendedError());
        return 2;
    }

    VideoStream depth, color;

    if (device.getSensorInfo(SENSOR_DEPTH) != NULL)
    {
        rc = depth.create(device, SENSOR_DEPTH);
        if (rc == STATUS_OK)
        {
            VideoMode depth_mode = depth.getVideoMode();
            depth_mode.setFps(30);
            depth_mode.setResolution(640, 480);
            depth_mode.setPixelFormat(PIXEL_FORMAT_DEPTH_100_UM);
            depth.setVideoMode(depth_mode);

            rc = depth.start();
            if (rc != STATUS_OK)
            {
                printf("Couldn't start the color stream\n%s\n", OpenNI::getExtendedError());
            }
        }
        else
        {
            printf("Couldn't create depth stream\n%s\n", OpenNI::getExtendedError());
        }
    }

    if (device.getSensorInfo(SENSOR_COLOR) != NULL)
    {
        rc = color.create(device, SENSOR_COLOR);
        if (rc == STATUS_OK)
        {
            VideoMode color_mode = color.getVideoMode();
            color_mode.setFps(30);
            color_mode.setResolution(640, 480);
            color_mode.setPixelFormat(PIXEL_FORMAT_RGB888);
            color.setVideoMode(color_mode);

            rc = color.start();
            if (rc != STATUS_OK)
            {
                printf("Couldn't start the color stream\n%s\n", OpenNI::getExtendedError());
            }
        }
        else
        {
            printf("Couldn't create color stream\n%s\n", OpenNI::getExtendedError());
        }
    }

    VideoFrameRef frame_depth;
    VideoFrameRef frame_color;

    VideoStream* streams[] = {&depth, &color};

    while (true)
    {
        int readyStream = -1;
        rc = OpenNI::waitForAnyStream(streams, 2, &readyStream, SAMPLE_READ_WAIT_TIMEOUT);
        if (rc != STATUS_OK)
        {
            printf("Wait failed! (timeout is %d ms)\n%s\n", SAMPLE_READ_WAIT_TIMEOUT, OpenNI::getExtendedError());
            break;
        }
            depth.readFrame(&frame_depth);
            color.readFrame(&frame_color);
        analyzeFrame(frame_depth,frame_color,depthImg, colorImg);
        //imshow("anh mau", colorImg);
        /////////////////////////////////////////////////////////////////////////////////////
        int width = 640;
		int height = 480;
		cvtColor(colorImg, imgGray, CV_RGB2GRAY);//xám hóa ?nh màu
		vector<string> codes;
		Mat corners;
		Rect roi(0, 3 * imgGray.rows / 4, imgGray.cols, imgGray.rows / 4); //c?t ?nh
		Mat imgROI = imgGray(roi);
		int widthSrc = imgROI.cols;
		int heightSrc = imgROI.rows;
		bilateralFilter(imgROI, denoise, 1, 1, 20);
		Mat gauss;
		GaussianBlur(denoise, gauss, Size(5, 5), 0, 0, BORDER_DEFAULT);
		Mat grad;
		cv::Canny(gauss, imgCanny, 10, 100, 3);
		cv::dilate(imgCanny, imgCanny, element);
		threshold(imgCanny, imgBin, 20, 255, THRESH_BINARY);
		Mat hough(imgROI.size(), CV_8U, Scalar(0));
		Mat result(imgBin.rows, imgBin.cols, CV_8U, Scalar(0));
		HoughLinesP(imgBin, lines, 2, CV_PI / 90, 100, 10, 30);
		DetectLanewithHought(lines, colorImg, result);
		imshow("line detect", result);

		if (left == 0) {
			for (int x = 0; x < widthSrc; x++)
			{
				if (colorImg.at<uchar>(10, x) == 255)left = x;
			}
		}

		if (right == 0) {
			for (int x = widthSrc; x >= 0; x--)
			{
				if (colorImg.at<uchar>(10, x) == 255)right = x;
			}
		}
		
		vector<Point> pointListLeft, pointListRight;
		if ((left != 0) && (right != 0))
			{
			//Xet ben trai
			for (int y = 0; y < heightSrc; y++)
			{
				for (int x = widthSrc / 2; x >= 0; x--)
				{
					if (result.at<uchar>(y, x) == 255 && x != left)
					{
						pointListLeft.push_back(Point(y, x));
						break;
					}
				}
			}

			//Xet ben phai
			for (int y = 0; y < heightSrc; y++)
				for (int x = widthSrc / 2; x < widthSrc; x++)
				{
					if (result.at<uchar>(y, x) == 255 && x != right)
					{
						pointListRight.push_back(Point(y, x));
						break;
					}
				}
			}
			int xL = 0, yL = 0, xR = 0, yR = 0;
			int xTam = 0, yTam = 0;
  			if (pointListLeft.size() > 0 && pointListRight.size() > 0)
			{
				for (int i = 0; i < pointListLeft.size(); i++)
				{
					xL = xL + pointListLeft.at(i).y;
					yL = yL + pointListLeft.at(i).x;
				}
				for (int i = 0; i < pointListRight.size(); i++)
				{
					xR = xR + pointListRight.at(i).y;
					yR = yR + pointListRight.at(i).x;
				}
				xTam = (xL / pointListLeft.size() + xR / pointListRight.size()) / 2;
				yTam = (yL / pointListLeft.size() + yR / pointListRight.size()) / 2;
				result.at<uchar>(yTam, xTam) = 255;
				yTam = yTam + height * 3 / 4;
			}
			else {
				if (ansX.size() == 0 || ansY.size()==0)
				{
					xTam = widthSrc / 2;
					yTam = 7*heightSrc/2; 
				}
				else
				{
					xTam = ansX.at(ansX.size() - 1);
					yTam = ansY.at(ansY.size() - 1);
				}
				
			}
		ansX.push_back(xTam); ansY.push_back(yTam);
		int X=widthSrc-xTam;
		//sprintf(buf_send, "%d\n", X);
		circle(colorImg, Point(xTam, yTam), 2, Scalar(255, 255, 0), 3);
		imshow("point detected ", colorImg);
		//imshow("canny ", imgCanny);
		//sprintf(buf_send, "%d",X);
		//printf("%s",buf_send);
		//api_uart_write(cport_nr, buf_send);
		//output_cap.write(colorImg);
 ////////////////////////////////////////   
        char key = waitKey(1);

        if( key == 27 ) break;

    }

    depth.stop();
    color.stop();
    depth.destroy();
    color.destroy();
    device.close();
    OpenNI::shutdown();

    return 0;
}
