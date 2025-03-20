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
	//��1��������ͷ������Ƶ  
	//VideoCapture capture(2);//����������ͷ��û�д򿪣�0��Ĭ��camera��realsense����Ĳ�ɫcamera��2�����豸���������������£���һ��Ϊ0��������Ϊ2��
	//if(!capture.isOpened())   {cout<< "cannot open the camera.";cin.get();return -1;} 
	Mat frame; //����һ��Mat���������ڴ洢ÿһ֡��ͼ��
	FILE *fp;//����FILEָ�룬��Ҫ��C/C++Ԥ�������еĶ��������ϣ�_CRT_SECURE_NO_WARNINGS
	FILE *fp2;//��ɫ�ļ���
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
		//capture >> frame;  //��ȡ��ǰ֡   
		

		if (frame.empty())
		{
			printf("--(!) No captured frame -- Break!");
			//break;                  
		}
		else
		{			
			
			setMouseCallback("Rect��ȡ", on_mouse, &frame);
			if (flag)
			  {
				rectangle(frame, startpoint, endpoint, Scalar::all(0), 2, 8, 0);
			  }

			imshow("Rect��ȡ", frame); //��ʾ��ǰ֡  

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

		//char c = cvWaitKey(33);//��ȡ�������룬Ҳ�ǿ�����Ƶ�Ĳ����ٶ�
		char c = waitKey(30); //��ʱ30ms,�ϱ��Ǿ���Թ�Ҳ����
		if (c == 27) //ESC�˳�
			break;
	}
/*	if (flag)//��αȽ�����UVֵ��ȡ���Ƶ��㷨ʧ�ܣ���tex_coords[i].u��tex_coords[i].v��0.0��1.0֮�䲢��������UV��������ϵ��ͼƬ���صĲ�ͬ
	 {		
		auto vertices = points.get_vertices();              // get vertices
		auto tex_coords = points.get_texture_coordinates(); // and texture coordinates,�����Ѿ�ת���ҹ�һ�����꣬�����½�Ϊ��0.0��0.0����
		//���Ͻ�Ϊ�յ㣨1.0��1.0���������������ȡ����������ϵ��ͬ���������������Ͻ�Ϊ��0,0��,���½�Ϊ�յ㣨width,height����
		//�������±Ƚ������ȡ���Ƶ��㷨ʧ��
		for (int i = 0; i < points.size(); i++)//�����½ǿ�ʼ���г���ɨ�裬���������Ͻǵ�
		//for (int i = 0; i <100; i++)
		 {
		   if (640*tex_coords[i].u > startpoint.x && 640*tex_coords[i].u < endpoint.x)
		    {
		      if (480*tex_coords[i].v > startpoint.y && 480*tex_coords[i].v < endpoint.y)
		        {
		           if (vertices[i].z > 0.2&&vertices[i].z < 2)
			        {
				      //fprintf(fp, "��������(%f,%f) ", tex_coords[i].u, tex_coords[i].v);
			         //fprintf(fp, " ��Ӧ��ά(%f,%f,%f)\n", vertices[i].x, vertices[i].y, vertices[i].z);
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
		for (int V = startpoint.y; V <= endpoint.y; V++)//һ��һ��ɨ�裬�����������
		{
			for (int U = startpoint.x; U <= endpoint.x; U++)
			 {
				upixel[0] = U/640.0;
				upixel[1] = V/480.0;
				// Query the distance from the camera to the object in the center of the image
				float udist = depth2.get_distance(U,V);
//				rs2_deproject_pixel_to_point(upoint, &intr, upixel, udist);//����Ҫ��upixelΪ������������ֻ����������U,V�����ˡ�
				// Print the distance 
				if (udist>0.2 && udist<2)// ��������ֻ�ܷ����ھ��뾵ͷ0.2��2m֮�䣬�ɸ���ʵ��������������������ȡ��Χ
				 {
//					fprintf(fp, "UV����(%f,%f) ", upixel[0], upixel[1]);//���ﲻ����%d����ȫ��0.
//					fprintf(fp, " ��Ӧ��ά(%f,%f,%f)\n", upoint[0], upoint[1], upoint[2]);
					fprintf(fp, "%f,%f,%f\n", upixel[0], upixel[1], udist);
					//��øõ�RGBֵ����ɫ����	
					IplImage* img =&(IplImage)frame;
					ptr = cvPtr2D(img, V, U, NULL);//����y����y�ᣨ��y�У�����height��x����x�ᣨ��x�У�����width
					fprintf(fp2,"%f,%f,%f\n", ptr[2]/255.0, ptr[1]/255.0, ptr[0]/255.0);//OPENCV��˳��BGR,����ע�������Ųŵõ�R,G,B
				 }
			 }
		 }
	 flag = false;
	}
	fclose(fp);
	fclose(fp2);
	cvDestroyWindow("Rect��ȡ");
	
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