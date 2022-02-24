#ifndef REALSCENE_CAM_OPT_H
#define REALSCENE_CAM_OPT_H
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_processing.hpp>
#include <librealsense2/hpp/rs_types.hpp>
#include <librealsense2/hpp/rs_sensor.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <iostream>


using namespace std;
using namespace cv;


struct Intrinsics
{
  float         ppx;
  float         ppy;
  float         fx;
  float         fy;
  float         coeffs[5];
  bool          distortion;
};

class CameraOpt_mivg
{
public:

    CameraOpt_mivg();
    ~CameraOpt_mivg();

    enum align_way{DepthToColor,ColorToDepth};
    enum TypeOfIntrinscis{DepthIntrinsics,ColorIntrinsics};

    vector<Mat> getCamerFrame(align_way way);
    void getIntrinsics(TypeOfIntrinscis type,align_way way);
    //重新投影图像的像素点，用于除去畸变参数
    void Project_Point_To_Pixel(float pixel[2], const struct Intrinsics * intrin, const float point[3]);


private:
    rs2::context ctx;
    char serial_number[100];
    rs2::pipeline pipe;
    rs2::frameset frames;
};


#endif
