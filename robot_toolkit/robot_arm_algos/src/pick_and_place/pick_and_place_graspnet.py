"""
Pick n Place example with one robot and one camera with known robot camera extrinsics.

Author: Ruthrash Hari
Date: 2/10/23 
"""
import numpy as np
import cv2
import matplotlib.pyplot as plt
import open3d as o3d
import dataclasses
from ..robot_camera_calibration._calibration_data_utils import tf_from_rvectvec, rvectvec_from_tf
from ..logger import logger
from ..config_reader import read_yaml_file

def get_offset_from_pose(pose, offset_dist = 0.15, offset_in_obj_frame = False):
    offsetvect_poseframe = np.array([0.0, 0.0, offset_dist])
    offset_pose = pose.copy()  
    if offset_in_obj_frame:
        grasp2robotbase_rot = offset_pose[0:3, 0:3]
        offsetvect_robotbaseframe = np.matmul(grasp2robotbase_rot, -1.0 * offsetvect_poseframe)
    else:
        offsetvect_robotbaseframe = offsetvect_poseframe
    offset_pose[0:3,-1] += offsetvect_robotbaseframe
    return offset_pose

def plot_image(image, blocking = True, transient = False, transient_time = 3.5):
    plt.imshow(image)
    plt.show(block=blocking)
    if transient:
        plt.pause(transient_time)
        plt.close() 
        
def draw_estimated_frame(color_image, camera, rvec, tvec):
    cv2.drawFrameAxes(color_image, camera.camera_matrix, camera.dist_coeffs, rvec, tvec, 0.05)
    return color_image    

class PickAndPlaceGraspNet:
    def __init__(self, robot_arm_object, camera, cam_extrinsics, camera_in_hand = True):
        self.robot_arm = robot_arm_object
        self.camera = camera
        self.cam_extrinsics = cam_extrinsics #either cam2base: camera in environment or cam2gripper: camera in hand
        self.camera_in_hand = camera_in_hand 
        
    def get_cam2base_tf(self,):
        if(self.camera_in_hand):        
            ee2base_tf = self.robot_arm.get_ee_frame()
            return np.matmul(ee2base_tf, self.cam_extrinsics)
        else: 
            return self.cam_extrinsics
    
    def pick_object(self, grasp_pose_camera_frame):
        grasp_pose = self.convert_grasp_pose(grasp_pose_camera_frame)
        pre_grasp_pose = get_offset_from_pose(pose = grasp_pose, offset_in_obj_frame = True)
        post_grasp_pose = get_offset_from_pose(pose = grasp_pose, offset_in_obj_frame = True)
        self.robot_arm.open_gripper()
        self.robot_arm.go_to_ee_pose(pre_grasp_pose)
        self.robot_arm.go_to_ee_pose(grasp_pose)
        self.robot_arm.close_gripper() #could test _soft grasp
        self.robot_arm.go_to_ee_pose(post_grasp_pose)
        return grasp_pose
    
    def place_object(self, place_pose):
        
        place_pose[2,3] += 0.05
        preplace_pose = get_offset_from_pose(pose = place_pose, offset_in_obj_frame = True)
        postplace_pose = get_offset_from_pose(pose = place_pose, offset_in_obj_frame = True)
        self.robot_arm.go_to_ee_pose(preplace_pose)
        self.robot_arm.go_to_ee_pose(place_pose)
        self.robot_arm.open_gripper()
        self.robot_arm.go_to_ee_pose(postplace_pose)
        return place_pose
        
    def convert_grasp_pose(self, grasp_pose_camera_frame, visualize_grasp = True):
        if visualize_grasp:
            plot_grasp_frames = []
            frame_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2).transform(grasp_pose_camera_frame)
            plot_grasp_frames.append(frame_mesh)
                
            rgb_image, depth_image = self.camera.get_current_rgbd_frames()
            pcd = self.camera.get_pointcloud_rgbd(rgb_image, depth_image)
            plot_grasp_frames.append(pcd)
            o3d.visualization.draw_geometries(plot_grasp_frames)

        grasp_pose_in_base = np.matmul(self.get_cam2base_tf(), grasp_pose_camera_frame) #use best pose  
        return grasp_pose_in_base