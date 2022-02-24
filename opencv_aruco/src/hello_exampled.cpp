#include <librealsense2/rs.hpp>
#include<librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/hpp/rs_types.hpp>
#include <librealsense2/hpp/rs_sensor.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace std;
using namespace cv;

int main()
{
	rs2::pipeline pipe;     //Contruct a pipeline which abstracts the device
	rs2::config cfg;    //Create a configuration for configuring the pipeline with a non default profile
	cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
	rs2::pipeline_profile selection = pipe.start(cfg);
	bool stop = false;
	while (!stop)
	{
		rs2::frameset frames;
		frames = pipe.wait_for_frames();
		//Get each frame
		auto color_frame = frames.get_color_frame();
		auto depth_frame = frames.get_depth_frame();
		//create cv::Mat from rs2::frame
		Mat color(Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
		imshow("Display Image", color);
		if (waitKey(10) >= 0)
			stop = true;
	}
	return 0;

}