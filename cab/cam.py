#!/home/ziye01/miniconda3/envs/cg/bin/python3
import sys
#sys.path.append("/home/jaco/kinova/kinova_ws/src/kinova_client/scripts")
sys.path.append("/home/ziye01/lees_ros/kinova_ws/src/kinova_client/scripts")
from csv import reader
import pyrealsense2 as rs
import numpy as np
import cv2
import sys
import csv
from read_kinova import mvig_kinova_reader

def batch_quat_to_mat(quat):
    """Convert quaternion coefficients to rotation matrix.
    For batch size = 1, batch_quat_to_mat achieve comparable speed
    with transforms3d.quaternions.quat2mat. For large batch size,
    batch_quat_to_mat is about 30 times faster than
    transforms3d.quaternions.quat2mat.

    Args:
        quat: numpy array of batch_size x 4.
    Returns:
        Rotation numpy array of batch_size x 3 x 3,
                 matrix corresponding to the quaternion.
    """

    batch_size = quat.shape[0]
    quat = quat / np.linalg.norm(quat + 1e-8, axis=1, keepdims=True)
    w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    w2, x2, y2, z2 = w ** 2, x ** 2, y ** 2, z ** 2
    wx, wy, wz = w * x, w * y, w * z
    xy, xz, yz = x * y, x * z, y * z

    rotMat = np.stack(
        [
            w2 + x2 - y2 - z2,
            2 * xy - 2 * wz,
            2 * wy + 2 * xz,
            2 * wz + 2 * xy,
            w2 - x2 + y2 - z2,
            2 * yz - 2 * wx,
            2 * xz - 2 * wy,
            2 * wx + 2 * yz,
            w2 - x2 - y2 + z2,
        ],
        axis=1,
    ).reshape(batch_size, 3, 3)
    return rotMat


def rodrigues_trans2trmat(tcp_r,tcp_t):
    H=np.zeros((4,4))
    H[:3,:3]=tcp_r
    H[:3,3]=tcp_t
    H[3,3]=1
    return H


class calibration(object):
    def __init__(self,mtx,pattern_size=(7,4),square_size=0.035,hand_eye="EIH"):
        self.mtx=mtx       #内部参数
        self.pattern_size=pattern_size  #角点长宽
        self.square_size=square_size    #方格长宽单位m
        self.hand_eye=hand_eye          #？
        self.pose_list=[]               #？
        self.init_calib()               #
        self.externMat=[]               #外部参数
    
    def init_calib(self):
        self.objp=np.zeros((self.pattern_size[0]*self.pattern_size[1],3),np.float32)
        self.objp[:,:2]=self.square_size*np.mgrid[0:self.pattern_size[0],0:self.pattern_size[1]].T.reshape(-1,2)
        #print(self.objp)
        for i in range(self.pattern_size[0]*self.pattern_size[1]):
            x,y=self.objp[i,0],self.objp[i,1]
            self.objp[i,0],self.objp[i,1]=y,x # 先进行行，后进行类
        # Arrays to store object points and image points from all the images
        self.objpoints=[]   # self.objpoints.append(self.objp) 计算外部矩阵时，需要使用的容器
        self.imgpoints=[]   # 角点列表，外面再套一层次
        self.images=[]      # 图像列表
        self.criteria=(cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER,30,0.001)

    
    def detectFeature(self,color,show=True):
        img=color
        # 转变成灰度图，计算亚像素时需要使用
        self.gray=cv2.cvtColor(img,cv2.COLOR_RGB2GRAY)
        #cv2.imshow('findCorners',self.gray)
        # 找出角点
        # img 彩色图
        # pattern_size [8,6] w=8,h=6  [0,0] [0,1] .... 回头用方向验证下,按照上面的self.obj,那么方向是先每一列的
        # CV_CALIB_CB_ADAPTIVE_THRESH：该函数的默认方式是根据图像的平均亮度值进行图像 二值化，设立此标志位的含义是采用变化的阈值进行自适应二值化
        ret,corners=cv2.findChessboardCorners(img,self.pattern_size,None,
                                              cv2.CALIB_CB_ADAPTIVE_THRESH)
        self.objpoints.append(self.objp)
        corners2=corners
        # 获取亚像素角点
        if ret==True:
           
            if(cv2.__version__).split('.')=='2':
                #cv::InputArray image, // 输入图像
                #cv::InputOutputArray corners, // 角点（既作为输入也作为输出）
                #cv::Size winSize, // 区域大小为 NXN; N=(winSize*2+1)
                #cv::Size zeroZone, // 类似于winSize，但是总具有较小的范围，Size(-1,-1)表示忽略, 死区
                #cv::TermCriteria criteria // 停止优化的标准
                cv2.cornerSubPix(self.gray,corners,(11,11),(-1,-1),self.criteria)
                corners2=corners
            else:
                corners2=cv2.cornerSubPix(self.gray,corners,(11,11),(-1,-1),self.criteria)
        # 得到角点的列表
        
        self.imgpoints.append(corners2)
        if show:
            #print(1111)
            # 画出角点来进行观察
            #cv2.drawChessboardCorners(img,self.pattern_size,corners2,ret)
            #cv2.imshow('findCorners',img)
            if cv2.waitKey(0)==ord('s'):
                cv2.destroyAllWindows()

    
    def detectAllFeature(self):
        for i in range(len(self.images)):
            #cv2.imshow("display",self.images[i])
            self.detectFeature(self.images[i])

    def rodrigues_trans2tr(self,rvec, tvec):
        r, _ = cv2.Rodrigues(rvec)
        tvec.shape = (3,)
        T = np.identity(4)
        T[0:3, 3] = tvec
        T[0:3, 0:3] = r
        return T

    def cal(self,optimize=False):
        # 找出外部矩阵与内部矩阵
        print(self.objpoints)
        ret,mtx,dist,rvecs,tvecs=cv2.calibrateCamera(self.objpoints,self.imgpoints,self.gray.shape[::-1],self.mtx,None)
        Hg2c=[]
        pose_list=np.array(self.pose_list)  # 这张列表里面为 w*h* shape(H)的矩阵
        for i in range(len(rvecs)):
            #tt=self.rodrigues_trans2tr(rvecs[i],tvecs[i]/1000.)
            tt=self.rodrigues_trans2tr(rvecs[i],tvecs[i])   # 将旋转举证与位移举证变成H举证
            Hg2c.append(tt)                                 
            self.externMat.append(tt)

        Hg2c=np.array(Hg2c)
        
        rot,pos=cv2.calibrateHandEye(pose_list[:,:3,:3],pose_list[:,:3,3],Hg2c[:,:3,:3],Hg2c[:,:3,3],method=cv2.CALIB_HAND_EYE_PARK)
        camT=np.identity(4)
        camT[:3,:3]=rot
        camT[:3,3]=pos[:,0]
        Hc2m=camT
        return Hc2m
    
    def averageRotation(self,R):
        n=len(R)
        tmp=np.zeros((3,3),dtype=R[0].dtype)
        for i in range(n):
            tmp=tmp+R[i]
        Rbarre=tmp/n
        RTR=np.dot(Rbarre.T,Rbarre)   # 把所有图片的旋转矩阵的batch_size 通道加起来，再求他们的ATA
        [D,V]=np.linalg.eig(RTR)  # 计算出特征值和特征分解 D，特征值，V特征向量
        D=np.diag(np.flipud(D))   # 将特征矩阵上下反转，如果一维向量，还是左右
        V=np.fliplr(V)
        sqrtD=np.sqrt(np.linalg.inv(D))  # 对特征举证的各项求倒数，然后求平方根
        if np.linalg.det(Rbarre[0:3,0:3])<0:
            sqrtD[2,2]=-1*sqrtD[2,2]
        temp=np.dot(V,sqrtD)           # 乘以新生成的V*sqartD 
        temp=np.dot(temp,V.T)          # V*sqartD&V.T
        Rvag=np.dot(Rbarre,temp)       # Rbarre*V*sqartD&V.T
        return Rvag                    # 他这一顿操作之后 不是R@RT@R=R？
    
    def averageTransformation(self,H):
        '''
        
        #:param H: [h] list, h is 4*4 pose.(np.array())
        #:return: average pose h
      
    '''
        n=len(H)
        Ravg=[]
        for i in range(n):
            Ravg.append(H[i][0:3,0:3])
        R=self.averageRotation(Ravg)
        Tavg = 0
        for i in range(n):
            Tavg+=H[i][0:3,3]
        Havg=np.zeros((4,4))
        Havg[0:3,0:3]=R
        Havg[0:3,3]=Tavg/n
        Havg[3,3]=1
        return Havg

def get_pos_rot_from_xyzq(xyzq):
    pos = np.array([xyzq[0], xyzq[1], xyzq[2]])     #x,y,z
    rot = batch_quat_to_mat(np.array([[xyzq[6],xyzq[3], xyzq[4], xyzq[5]]]))[0]   # w,x,y,z
    return pos, rot 

class CameraL(object):
    def __init__(self,indexCam=0):
        self.pipeline=rs.pipeline()   # 配置管道流
        self.config=rs.config()       # 生成配置容器
        self.config.enable_stream(rs.stream.depth,1280,720,rs.format.z16,30)  # fps=30 深度是16位整形
        self.config.enable_stream(rs.stream.color,1280,720,rs.format.bgr8,30) # fps=3- bgr8 ，30
        self.align_to=rs.stream.color
        self.align=rs.align(self.align_to)  # 对齐颜色空间，给颜色空间加入深度信息

        # 获取想要的详解
        connect_device=[]
        for d in rs.context().devices:
            if d.get_info(rs.camera_info.name).lower() != 'platform camera':
                connect_device.append(d.get_info(rs.camera_info.serial_number))
                
        self.config.enable_device(connect_device[0])
        self.pipeline_proflie=self.pipeline.start(self.config)
        self.device=self.pipeline_proflie.get_device()
        advanced_mode=rs.rs400_advanced_mode(self.device)
        self.mtx,_=self.getIntrinsics()
        #with open(r"config/d435_high_accuracy.json", 'r') as file:
        #    json_text = file.read().strip()
        #advanced_mode.load_json(json_text)
        
        self.hole_filling=rs.hole_filling_filter()

        align_to=rs.stream.color
        self.align=rs.align(align_to)

        # cam init
        print('cam init ...')
        i=60
        while i>0:
            frames=self.pipeline.wait_for_frames()
            aligned_frames=self.align.process(frames) #对其颜色信息
            depth_frame=aligned_frames.get_depth_frame() 
            color_frame=aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            depth_image=np.asanyarray(depth_frame.get_data())
            color_image=np.asanyarray(color_frame.get_data())
            i-=1
        print("cam init done.")

    
    def inpaint(self,img,missing_value=0):
        '''
        pip opencv-python == 3.4.8.29
        :param image:
        '''
        pass

    def get_depth_scale(self):
        return self.device.first_depth_sensor().get_depth_scale()  # 深度乘以当前值 

    def get_data(self,hole_filling=False):
        while True:
            frames=self.pipeline.wait_for_frames()
            aligned_frames=self.align.process(frames)
            depth_frame=aligned_frames.get_depth_frame()
            if hole_filling:
                depth_frame=self.hole_filling.process(depth_frame)
            color_frame=aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue
            depth_image=np.asanyarray(depth_frame.get_data())
            color_image=np.asanyarray(color_frame.get_data())
            break
        return color_image,depth_image,color_frame,depth_frame
    
    def getIntrinsics(self):
        frames=self.pipeline.wait_for_frames()
        aligned_frames=self.align.process(frames)
        color_frame=aligned_frames.get_color_frame()
        intrinsics=color_frame.get_profile().as_video_stream_profile().get_intrinsics()
        mtx=[intrinsics.width,intrinsics.height,intrinsics.ppx,intrinsics.ppy,intrinsics.fx,intrinsics.fy]
        camIntrinsics=np.array([[mtx[4],0,mtx[2]],
                                [0,mtx[5],mtx[3]],
                                [0,0,1.]])
        return camIntrinsics,mtx
        

if __name__=="__main__":
    cam=CameraL()
    calib=calibration(cam.mtx,pattern_size=(7,4))
    '''
    rows=[]
    count=1
    while count<11:
        color,depth,_,_=cam.get_data()
        cv2.imshow("vis",color)
        action=cv2.waitKey(30)
        rd=mvig_kinova_reader()
        
        if action &0xFF==ord('q'):
            break
        if action &0xFF==ord('s'):
            print("saving the {}-th data".format(count))
            row=rd.read_joint_pose()
            print("row")
            print(row)
            rows.append(row)
            print("pose")
            print(rd.read_tool_pose())
            pos, rot = get_pos_rot_from_xyzq(rd.read_tool_pose())
            np.save('../save/t/t_{}.npy'.format(count), pos)
            np.save('../save/r/r_{}.npy'.format(count), rot)
            cv2.imwrite('../save/img/{}.jpg'.format(count), color)
            count+=1
            continue
            
        
    with open("../save/csv.txt", 'w+', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerows(rows)
    '''
    for i in range(10):
        temp_r=np.load('../save/r/r_{}.npy'.format(i+1))
        temp_t=np.load('../save/t/t_{}.npy'.format(i+1))
        temp=rodrigues_trans2trmat(temp_r,temp_t)
        imgtemp=cv2.imread("../save/img/{}.jpg".format(i+1))
        calib.images.append(imgtemp)
        calib.pose_list.append(temp)
    print("11111111111111111111111111")
    calib.detectAllFeature()
    
    print("=====================Flexiv_hand_eye===========================")
    H2C=calib.cal()
    np.save('../save/kinovaH2E.npy',H2C)
    print(H2C)
    