import open3d as o3d
import pyrealsense2 as rs
import numpy as np
import cv2
import sys
sys.path.append("..")
#import matplotlib.pyplot as plt
from cam import CameraL





if __name__=="__main__":
    cam=CameraL()
    vis = o3d.visualization.Visualizer()
    vis.create_window()

    color,depth,color_f,depth_f=cam.get_data()
    depth_scale=cam.get_depth_scale()
    img_depth = o3d.geometry.Image(depth)
    img_color = o3d.geometry.Image(color)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(img_color, img_depth)
    intrix,intrinsics=cam.getIntrinsics()
    pinhole_camera_intrinsic = o3d.camera.PinholeCameraIntrinsic(
        intrinsics[0], intrinsics[1], intrinsics[4], intrinsics[5], intrinsics[2], intrinsics[3])
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd,pinhole_camera_intrinsic)
    pcd =pcd.voxel_down_sample(voxel_size=0.1)
    print(pcd.points)
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    vis.add_geometry(pcd)

    while True:
        vis.remove_geometry(pcd)
        color,depth,color_f,depth_f=cam.get_data()
        depth_scale=cam.get_depth_scale()
        img_depth = o3d.geometry.Image(depth)
        img_color = o3d.geometry.Image(color)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(img_color, img_depth)
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd,pinhole_camera_intrinsic)
        pcd =pcd.voxel_down_sample(voxel_size=0.003)

        vis.add_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        cv2.imshow("vis",color)
        action=cv2.waitKey(30)
        if action & 0xFF ==ord('q'):
            break
   
    