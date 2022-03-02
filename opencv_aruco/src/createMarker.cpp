#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

using namespace std;
using namespace cv;

int main()
{
    cv::Mat markerImage;
    //字典由250个maker组成，每个marker的大小为6x6
    cv::Ptr<cv::aruco::Dictionary> dictionary=cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    /*
     * cv::aruco::drawMarker
     * 第一个参数是之前创建的Dictionary对象。
     * 
     * 第二个参数是marker的id，在这个例子中选择的是字典DICT_6X6_250 的第23个marker。注意到每个字典是由不同数目的Marker组成的，
     * 在这个例子中的字典中，有效的Id数字范围是0到249。不在有效区间的特定id将会产生异常。
     * 
     * 第三个参数，200，是输出Marker图像的大小。在这个例子中，输出的图像将是200x200像素大小。注意到这一参数需要满足能够存储特定字典 的所有位。
     * 举例来说，你不能为6x6大小的marker生成一个5x5图像（这还没有考虑到Marker的边界）。除此之外，为了避免变形，这一参数最好和位数+边界的大小成正比，或者至少要比marker的大小大得多（如这个例子中的200)，这样变形就不显著了
     * 
     * 第四个参数是输出的图像。
     * 最终，最后一个参数是一个可选的参数，它指定了Marer黑色边界的大小。这一大小与位数数目成正比。例如，值为2意味着边界的宽度将会是2的倍数。默认的值为1。
    */
    cv::aruco::drawMarker(dictionary,2,200,markerImage,1);
    imwrite("./aruco_tag_2.png",markerImage);
    imshow("test",markerImage);
    waitKey();
    return 0;
}