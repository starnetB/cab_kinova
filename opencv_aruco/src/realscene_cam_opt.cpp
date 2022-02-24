#include <realscene_cam_opt.hpp>

/*

class CameraOpt
{
public:
    CameraOpt();
    ~CameraOpt();
    void getCamerFrame();
    void getIntrinsics();
    void getDistortion();
    void getColor2DepthMat();

private:
    rs2::context ctx;
    char serial_number[100];
    rs2::config cfg;
    rs2::pipline pip;
    
    rs2::stream_profile dprofile,cprofile;
    d
};*/

CameraOpt_mivg::CameraOpt_mivg()
{
    auto devs=this->ctx.query_devices();
    int device_num=devs.size();
    std::cout<<"device num"<<device_num<<std::endl;
    rs2::device dev=devs[0];
    
    //memset(this->serial_number,0,sizeof(char)*1000);
    strcpy(this->serial_number, dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
    //printf("serial_number: %s\n",serial_number);
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280,720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280,720, RS2_FORMAT_BGR8, 30);
    rs2::pipeline pipe;
    this->pipe=pipe;
    this->pipe.start(cfg);   
}
CameraOpt_mivg::~CameraOpt_mivg(){}

vector<Mat> CameraOpt_mivg::getCamerFrame(align_way way)
{
    //深度图对齐器
	rs2::align align_to_depth(RS2_STREAM_DEPTH);
	//彩色图对齐器具
	rs2::align align_to_color(RS2_STREAM_COLOR);

    this->frames=this->pipe.wait_for_frames();///等待一帧数据，默认等待5s
    if(way==DepthToColor)
        this->frames=align_to_color.process(this->frames);
    if(way==ColorToDepth)
        this->frames=align_to_depth.process(this->frames);

    rs2::depth_frame depth= this->frames.get_depth_frame();
    rs2::video_frame color= this->frames.get_color_frame();
    int color_width =color.get_width();
    int color_height=color.get_height();

    int depth_width=depth.get_width();
    int depth_height=depth.get_height();

    ///将彩色图像和深度图像转换为Opencv格式
    cv::Mat image(cv::Size(color_width, color_height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat depthmat(cv::Size(depth_width,depth_height),CV_16U,(void*)depth.get_data(),cv::Mat::AUTO_STEP);

    vector<Mat> imgVector;
    imgVector.push_back(image);
    imgVector.push_back(depthmat);

    return imgVector;
    //显示
    //cv::imshow("image",image);
    //cv::imshow("depth",depthmat);
    //dcv::waitKey(1);
}


void CameraOpt_mivg::Project_Point_To_Pixel(float pixel[2], const struct Intrinsics * intrin, const float point[3])
{
    float x = point[0] / point[2], y = point[1] / point[2];
    
    if(intrin->distortion){
        float r2  = x*x + y*y;
        float f = 1 + intrin->coeffs[0]*r2 + intrin->coeffs[1]*r2*r2 + intrin->coeffs[4]*r2*r2*r2;
        x *= f;
        y *= f;
        float dx = x + 2*intrin->coeffs[2]*x*y + intrin->coeffs[3]*(r2 + 2*x*x);
        float dy = y + 2*intrin->coeffs[3]*x*y + intrin->coeffs[2]*(r2 + 2*y*y);
        x = dx;
        y = dy;
    }

    pixel[0] = x * intrin->fx + intrin->ppx;
    pixel[1] = y * intrin->fy + intrin->ppy;
}

void CameraOpt_mivg::getIntrinsics(TypeOfIntrinscis type,align_way way)
{
   
     //深度图对齐器
	rs2::align align_to_depth(RS2_STREAM_DEPTH);
	//彩色图对齐器具
	rs2::align align_to_color(RS2_STREAM_COLOR);
   
    this->frames=this->pipe.wait_for_frames();///等待一帧数据，默认等待5s
    if(way==DepthToColor)
        this->frames=align_to_color.process(this->frames);
    if(way==ColorToDepth)
        this->frames=align_to_depth.process(this->frames);
   
    rs2::depth_frame depth= this->frames.get_depth_frame();
    rs2::video_frame color= this->frames.get_color_frame();
    rs2::stream_profile dprofile=depth.get_profile();
    rs2::stream_profile cprofile=color.get_profile();

    //获取彩色相机内参
    if(type==DepthIntrinsics)
    {
        rs2::video_stream_profile dvsprofile(dprofile);
        rs2_intrinsics depth_intrin =  dvsprofile.get_intrinsics();
        std::cout<<"\ndepth intrinsics: ";
        std::cout<<depth_intrin.width<<"  "<<depth_intrin.height<<"  ";
        std::cout<<depth_intrin.ppx<<"  "<<depth_intrin.ppy<<"  ";
        std::cout<<depth_intrin.fx<<"  "<<depth_intrin.fy<<std::endl;
        std::cout<<"coeffs: ";
        for(auto value : depth_intrin.coeffs)
            std::cout<<value<<"  ";
        std::cout<<std::endl;
        std::cout<<"distortion model: "<<depth_intrin.model<<std::endl;///畸变模型
    }
    if(type==ColorIntrinsics)
    {
        rs2::video_stream_profile cvsprofile(cprofile);
        rs2_intrinsics color_intrin=cvsprofile.get_intrinsics();

        std::cout<<"\n color intrinsics: ";
        std::cout<<color_intrin.width<<"  "<<color_intrin.height<<"  ";
        std::cout<<color_intrin.ppx<<"  "<<color_intrin.ppy<<"  ";
        std::cout<<color_intrin.fx<<"  "<<color_intrin.fy<<std::endl;
        std::cout<<"coeffs: ";
        for(auto value : color_intrin.coeffs)
            std::cout<<value<<"  ";
        std::cout<<std::endl;
        std::cout<<"distortion model: "<<color_intrin.model<<std::endl;///畸变模型
    }
}

