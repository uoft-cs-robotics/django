import numpy as np
from robot_arm_algos.src.robot_camera_calibration._calibration_data_utils import read_cam_robot_extrinsics
from robot_arm_algos.src.robot_arm.robot_frankapy import RobotFrankaPy
from robot_arm_algos.src.camera.realsense_camera import RealSenseCamera
from robot_arm_algos.src.pick_and_place.object import Object
from robot_arm_algos.src.pick_and_place.pick_and_place_graspnet import PickAndPlaceGraspNet
from robot_arm_algos.src.logger import logger
from robot_arm_algos.src.inference.contact_graspnet import ContactGraspNet
import robot_arm_algos.src.inference._contact_graspnet_config_utils as config_utils
import open3d as o3d

test_config = {
    "extrinsics_file" : "robot_arm_algos/robot_camera_extrinsics.yaml",
    "camera_in_hand": True,
    "checkpoint_dir": "/root/git/scratchpad/scene_test_2048_bs3_hor_sigma_0025",
    "use_depth": False,
    }

def main():  
    camera = RealSenseCamera()
    global_config = config_utils.load_config(test_config["checkpoint_dir"], batch_size=1, arg_configs=[])
    graspnet_object = ContactGraspNet(global_config, test_config["checkpoint_dir"])
    robot_arm_object = RobotFrankaPy(init_node = True, 
                                    with_franka_gripper = True)   
    extrinsics = read_cam_robot_extrinsics(extrinsics_file_name = test_config["extrinsics_file"])
    picknplace = PickAndPlaceGraspNet(robot_arm_object = robot_arm_object,
                              camera = camera, 
                              cam_extrinsics = extrinsics, 
                              camera_in_hand = test_config["camera_in_hand"])
    
    robot_arm_object.fpy_object.reset_joints()      

    # get grasps
    grasps, scores, contact_points = graspnet_object.generate_grasps(camera, use_depth_for_seg = test_config["use_depth"], is_visualize_grasps = False)
    grasps, scores = grasps[255.0], scores[255.0]
    idxs = np.argsort(scores)[:3]
    sel_grasps = grasps[idxs]
    scores = scores[idxs]

    base_grasps = [picknplace.convert_grasp_pose(grasp, visualize_grasp=False) for grasp in sel_grasps]
    z = np.array([0, 0, -1]).T
    verticality = [np.dot(z, grasp[:3, 2]) for grasp in base_grasps]
    final_score = [x*y for x,y in zip(verticality, scores)]

    # plot_grasp_frames = []
    # for plot_grasp, score in zip(sel_grasps, verticality):
    #     frame_mesh = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2).transform(plot_grasp)
    #     box = o3d.geometry.TriangleMesh.create_box(width=0.03, height=0.03, depth=0.03).transform(plot_grasp)
    #     box.paint_uniform_color([score, 0, 0])
    #     # frame_mesh.paint_uniform_color([score, 0, 0])
    #     plot_grasp_frames.append(frame_mesh)
    #     plot_grasp_frames.append(box)
        
    # rgb_image, depth_image = picknplace.camera.get_current_rgbd_frames()
    # pcd = picknplace.camera.get_pointcloud_rgbd(rgb_image, depth_image)
    # plot_grasp_frames.append(pcd)
    # o3d.visualization.draw_geometries(plot_grasp_frames)
    
    v_idxs = np.argsort(verticality)
    sorted_grasps = sel_grasps[v_idxs]
    print(verticality[v_idxs[-1]])
    grasp_pose_camera_frame = sorted_grasps[-1] #this returns the best grasp as rotation matrix (4x4)

    # start pick and place
    grasp_pose = picknplace.pick_object(grasp_pose_camera_frame)
    picknplace.place_object(place_pose = grasp_pose)
    robot_arm_object.fpy_object.reset_joints()      
          
if __name__ == "__main__":
    main()    