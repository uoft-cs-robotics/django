import yaml
import time

from move_group_python_kinova import MoveGroupPythonInterfaceTutorial
from ._robot_arm import RobotArm
from ..logger import logger

class RobotKinovaMoveIt(RobotArm):
    def __init__(self, group_name = "manipulator_right"):
        self.group_name = group_name
        self.arm_move_group = MoveGroupPythonInterfaceTutorial(group_name = group_name)
        self.arm_move_group.move_group.set_max_velocity_scaling_factor(0.8)
        self.arm_move_group.move_group.set_max_acceleration_scaling_factor(0.8)
        
        self.arm_move_group.go_to_ee_pose([0.5, 0.5, 0.5, 0, 0, 0, 1])
    
    
    def get_ee_frame(self,):
        pass
    
    def get_joint_angles(self,):
        pass
    
    def go_to_ee_pose(self, goal_ee_pose):
        self.arm_move_group.go_to_ee_pose(goal_ee_pose)
        return

    def go_to_ee_pose_delta(self, delta_ee_pose):
        print("not implemented yet")
        return
    
    
    def get_randomized_absolute_poses(self, n_poses, flag_camera_in_hand = True):
        initial_pose = self.fpy_object.get_pose()
        from itertools import cycle
        toggle_signage = cycle([-1.0000,1.0000])
        def toggle_sign():
            return toggle_signage.__next__()

        ee_poses = []
        ee_poses.append(initial_pose)
        r, p ,y = ros_tf_utils.euler_from_matrix(initial_pose.rotation, 'rxyz')
        initial_translation = initial_pose.translation
        z_grid = np.array([1.0, 0.0, -1.0]) * 0.05
        signage = [-1.0000,1.0000]
        goal_pose = initial_pose.copy()
        for i in range(n_poses):
            x_offset = i % int(n_poses/len(z_grid))
            y_offset = i % int(n_poses/len(z_grid))
            sign = toggle_sign()
            x_offset *= 0.01 * random.choice(signage)
            y_offset *= 0.01 * random.choice(signage)    
            z_offset = random.choice(z_grid)            
            r_new = r + math.radians(random.uniform(0.0,30.0)) * random.choice(signage)
            p_new = p + math.radians(random.uniform(0.0,30.0)) * random.choice(signage)
            y_new = y + math.radians(random.uniform(0.0,80.0)) * random.choice(signage)
            goal_pose = RigidTransform(from_frame='franka_tool', 
                                    to_frame='world',
                                    rotation=ros_tf_utils.euler_matrix(r_new, p_new, y_new, 'rxyz')[0:3,0:3],
                                    translation=initial_pose.translation + np.array([x_offset, y_offset, z_offset]))
            ee_poses.append(goal_pose)
            # print(math.degrees(r_new),math.degrees(p_new),math.degrees(y_new),x_offset,y_offset,z_offset)

        return ee_poses    