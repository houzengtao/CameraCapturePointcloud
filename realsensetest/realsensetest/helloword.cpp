#include<opencv2/opencv.hpp>   
#include "librealsense2/rs.hpp" 
#include "example.hpp"  
#include <librealsense2/rsutil.h>
#include "cv-helpers.hpp"
#include <algorithm>            // std::min, std::max
#include <iostream>
using namespace cv;
using namespace std;
using namespace rs2;
void on_mouse(int EVENT, int x, int y, int flags, void* userdata);
Point startpoint, endpoint;
const int width = 640;
const int height = 480;
const int fps = 30;
bool flag = false;
int main()
{
	rs2::align align_to(RS2_STREAM_COLOR);
	//【1】从摄像头读入视频  
	//VideoCapture capture(2);//若测试摄像头有没有打开，0是默认camera，realsense相机的彩色camera是2（看设备管理器，从上往下，第一个为0，第三个为2）
	//if(!capture.isOpened())   {cout<< "cannot open the camera.";cin.get();return -1;} 
	Mat frame; //定义一个Mat变量，用于存储每一帧的图像
	FILE *fp;//想用FILE指针，需要在C/C++预编译器中的定义栏加上：_CRT_SECURE_NO_WARNINGS
	FILE *fp2;//颜色文件的
	fp = fopen("pointcloud.txt", "w");
	fp2 = fopen("color.txt", "w");
	// Declare pointcloud object, for calculating pointclouds and texture mappings
	//pointcloud pc = rs2::context().operator std::shared_ptr<rs2_context>//create_pointcloud();
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;
 // Create a Pipeline, which serves as a top-level API for streaming and processing frames
	//pipeline p;
	rs2::pipeline p;//Pipeline
	rs2::config pipe_config;
	pipe_config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
	pipe_config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
	//	pipe_config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
	pipe_config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);

	rs2::pipeline_profile profile = p.start(pipe_config);
	frameset aligned_set2;
	// start the pipeline
	//p.start();
	startpoint.x = 0;
	startpoint.y = 0;
	endpoint.x = 0;
	endpoint.y = 0;
	while (1)
	{
		// Block program until frames arrive
		//frameset frames = p.wait_for_frames();
		// Try to get a frame of a depth image
		//depth_frame depth = frames.get_depth_frame();
		frameset data = p.wait_for_frames();
		// Make sure the frameset is spatialy aligned 
		// (each pixel in depth image corresponds to the same pixel in the color image)
		frameset aligned_set = align_to.process(data);
		aligned_set2 = aligned_set;
		depth_frame depth = aligned_set.get_depth_frame();
		// The frameset might not contain a depth frame, if so continue until it does
		if (!depth) continue;
		
		auto color = aligned_set.get_color_frame();
		auto color_mat = frame_to_mat(color);
		frame = color_mat;
		// Tell pointcloud object to map to this color frame
		pc.map_to(color);
		// Generate the pointcloud and texture mappings
		points = pc.calculate(depth);
		//capture >> frame;  //读取当前帧   
		

		if (frame.empty())
		{
			printf("--(!) No captured frame -- Break!");
			//break;                  
		}
		else
		{			
			
			setMouseCallback("Rect读取", on_mouse, &frame);
			if (flag)
			  {
				rectangle(frame, startpoint, endpoint, Scalar::all(0), 2, 8, 0);
			  }

			imshow("Rect读取", frame); //显示当前帧  

		}

		// Get the depth frame's dimensions
		//float width = depth.get_width();
		//float height = depth.get_height();
		 float width = startpoint.x + endpoint.x;				
		 float height = startpoint.y + endpoint.y;
		// Query the distance from the camera to the object in the center of the image
		float dist_to_center = depth.get_distance(width / 2, height / 2);

		// Print the distance 
		std::cout << "The camera is facing an object " << dist_to_center << " meters away \r";

		//char c = cvWaitKey(33);//获取键盘输入，也是控制视频的播放速度
		char c = waitKey(30); //延时30ms,上边那句测试过也可以
		if (c == 27) //ESC退出
			break;
	}
/*	if (flag)//这段比较区间UV值获取点云的算法失败，因tex_coords[i].u和tex_coords[i].v在0.0到1.0之间并不是像素UV，且坐标系跟图片像素的不同
	 {		
		auto vertices = points.get_vertices();              // get vertices
		auto tex_coords = points.get_texture_coordinates(); // and texture coordinates,它是已经转化且归一的坐标，以左下角为起（0.0，0.0），
		//右上角为终点（1.0，1.0）跟我们鼠标点击获取的像素坐标系不同，像素坐标以左上角为起（0,0）,右下角为终点（width,height），
		//所以以下比较区间获取点云的算法失败
		for (int i = 0; i < points.size(); i++)//从左下角开始逐行朝上扫描，结束于右上角点
		//for (int i = 0; i <100; i++)
		 {
		   if (640*tex_coords[i].u > startpoint.x && 640*tex_coords[i].u < endpoint.x)
		    {
		      if (480*tex_coords[i].v > startpoint.y && 480*tex_coords[i].v < endpoint.y)
		        {
		           if (vertices[i].z > 0.2&&vertices[i].z < 2)
			        {
				      //fprintf(fp, "纹理坐标(%f,%f) ", tex_coords[i].u, tex_coords[i].v);
			         //fprintf(fp, " 对应三维(%f,%f,%f)\n", vertices[i].x, vertices[i].y, vertices[i].z);
				      // fprintf(fp, "%f,%f,%f\n", vertices[i].x, vertices[i].y, vertices[i].z);
					   fprintf(fp, "%f,%f,%f\n", tex_coords[i].u, tex_coords[i].v,vertices[i].z);
			         }
		         }
		     }	
		 }
	   flag = false;
	 }*/

	if (flag)
	{
		float upixel[2]; // From pixel
		float upoint[3]; // From point (in 3D)
		uchar* ptr;
		depth_frame depth2 = aligned_set2.get_depth_frame();
//		rs2_intrinsics intr = depth2.get_profile().as<rs2::video_stream_profile>().get_intrinsics(); // Calibration data	
		for (int V = startpoint.y; V <= endpoint.y; V++)//一行一行扫描，从上往下输出
		{
			for (int U = startpoint.x; U <= endpoint.x; U++)
			 {
				upixel[0] = U/640.0;
				upixel[1] = V/480.0;
				// Query the distance from the camera to the object in the center of the image
				float udist = depth2.get_distance(U,V);
//				rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);//函数要求upixel为浮点数，所以只能无视整数U,V这样了。
				// Print the distance 
				if (udist>0.2 && udist<2)// 手术对象只能放在在距离镜头0.2到2m之间，可根据实际需求更改手术对象的提取范围
				 {
//					fprintf(fp, "UV坐标(%f,%f) ", upixel[0], upixel[1]);//这里不能用%d，会全成0.
//					fprintf(fp, " 对应三维(%f,%f,%f)\n", upoint[0], upoint[1], upoint[2]);
					fprintf(fp, "%f,%f,%f\n", upixel[0], upixel[1], udist);
					//获得该点RGB值即彩色纹理	
					IplImage* img =&(IplImage)frame;
					ptr = cvPtr2D(img, V, U, NULL);//其中y代表y轴（第y行），即height；x代表x轴（第x列），即width
					fprintf(fp2,"%f,%f,%f\n", ptr[2]/255.0, ptr[1]/255.0, ptr[0]/255.0);//OPENCV的顺序BGR,所以注意这样排才得到R,G,B
				 }
			 }
		 }
	 flag = false;
	}
	fclose(fp);
	fclose(fp2);
	cvDestroyWindow("Rect读取");
	
	return 0;
}
void on_mouse(int EVENT, int x, int y, int flags, void* userdata)
{
	Mat hh;
	hh = *(Mat*)userdata;
	Point p(x, y);
	switch (EVENT)
	{
	    case EVENT_LBUTTONDOWN:
	     {
		   printf("b=%d\t", hh.at<Vec3b>(p)[0]);
		   printf("g=%d\t", hh.at<Vec3b>(p)[1]);
		   printf("r=%d\n", hh.at<Vec3b>(p)[2]);
		   printf("DOWN x=%d\t", x);
		   printf("DOWN y=%d\n", y);
		   startpoint.x = x;
		   startpoint.y = y;
		   flag = false;
	     }
		case EVENT_LBUTTONUP:
		{
			printf("b=%d\t", hh.at<Vec3b>(p)[0]);
			printf("g=%d\t", hh.at<Vec3b>(p)[1]);
			printf("r=%d\n", hh.at<Vec3b>(p)[2]);
			printf("UP x=%d\t", x);
			printf("UP y=%d\n", y);
			endpoint.x = x;
			endpoint.y=y;
			flag = true;
		}
	   break;

	}
}