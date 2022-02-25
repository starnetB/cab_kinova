#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>   
#include <realscene_cam_opt.hpp>


int main()
{
    CameraOpt_mivg mvig;
    const cv::Ptr<cv::aruco::Dictionary> dictionary=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_250);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    bool showrejected=false;
    while(true)
    {
        cv::Mat imageCopy;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners,rejected;
        auto MVector=mvig.getCamerFrame(CameraOpt_mivg::DepthToColor);
    
        
        MVector[0].copyTo(imageCopy);
        
        cv::aruco::detectMarkers(imageCopy,dictionary,corners,ids,parameters,rejected);
        // if at least one marker detected 
        if (ids.size()>0)
        {
            std::cout<<ids.size()<<std::endl;
            cv::aruco::drawDetectedMarkers(imageCopy,corners,ids);
        }
        if(showrejected &&rejected.size() > 0)
        {
            std::cout<<rejected.size()<<std::endl;  
            aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));
        }
        cv::imshow("out",imageCopy);
        char key=(char)cv::waitKey(30);
        if(key==27) break;
    }

}