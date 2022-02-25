#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "opencv2/core/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <realscene_cam_opt.hpp>

using namespace std;
using namespace cv;

int main()
{
    double markerlength=0.03;
    CameraOpt_mivg mvig;
    rs2_intrinsics mvig_intrinsics=mvig.getIntrinsics(CameraOpt_mivg::DepthIntrinsics,CameraOpt_mivg::DepthToColor);
    cv::Mat intrinsics=(Mat_<double>(3,3)<<
                        mvig_intrinsics.fx,0.0,mvig_intrinsics.ppx,
                        0.0,mvig_intrinsics.fy,mvig_intrinsics.ppy,
                        0.0,    0.0      ,1             );
    
    cv::Mat distCoeffs=(cv::Mat_<double>(5,1)<<mvig_intrinsics.coeffs[0],mvig_intrinsics.coeffs[1],mvig_intrinsics.coeffs[2],mvig_intrinsics.coeffs[3],mvig_intrinsics.coeffs[4]);
    cv::Ptr<cv::aruco::Dictionary> dictionary=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);

    while (true)
    {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::Mat imageCopy;
        std::vector<cv::Mat> MVector=mvig.getCamerFrame(CameraOpt_mivg::DepthToColor);
        MVector[0].copyTo(imageCopy);
        cv::aruco::detectMarkers(imageCopy,dictionary,corners,ids);//检测靶标
        //if at least one marker detected
        
        if(ids.size()>0)
        {
            cv::aruco::drawDetectedMarkers(imageCopy,corners,ids);//绘制检测到的靶标
            for(unsigned int i=0;i<ids.size();i++)
            {
                std::vector<cv::Vec3d> rvecs,tvecs;
                 
                cv::aruco::estimatePoseSingleMarkers(corners, markerlength, intrinsics, distCoeffs, rvecs, tvecs);//求解旋转矩阵rvecs和平移矩阵tvecs
                //cv::aruco::drawAxis(imageCopy,intrinsics,distCoeffs, rvecs[i], tvecs[i], 0.1);
                //3.rotation vector to eulerAngles
                cv::Mat rmat;
                Rodrigues(rvecs[i],rmat);
                Eigen::Matrix3d rotation_matrix3d;
                cv2eigen(rmat,rotation_matrix3d);
                Eigen::Vector3d eulerAngle=rotation_matrix3d.eulerAngles(0,1,2);
                cout<<"pitch "<<eulerAngle.x()<<"yaw "<<eulerAngle.y()<<"roll"<<eulerAngle.z()<<endl;
                cout<<"x= "<<tvecs[i][0]<<"y="<<tvecs[i][1]<<"z="<<tvecs[i][2]<<endl;
            }
        }
        cv::imshow("out",imageCopy);
        char key=(char)cv::waitKey(30);
        if(key==27) break;
        
        
    }
    
    return 0;

}