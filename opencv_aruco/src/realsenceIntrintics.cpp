# include <realscene_cam_opt.hpp>
//当下程序 获取realscene相机内参和即便参数
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <iostream>
#include <opencv2/opencv.hpp> 


int main(int argc, char * argv[]) 
{
    CameraOpt_mivg mvig;
    mvig.getIntrinsics(CameraOpt_mivg::DepthIntrinsics,CameraOpt_mivg::DepthToColor);
}
