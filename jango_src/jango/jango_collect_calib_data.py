from move_group_python_kinova import MoveGroupPythonInterfaceTutorial
import yaml
import time

if __name__=="__main__":
    right_arm_group_name = "manipulator_right"
    left_arm_group_name = "manipulator_left"
    right_arm_group = MoveGroupPythonInterfaceTutorial(group_name = right_arm_group_name)
    
    left_arm_group = MoveGroupPythonInterfaceTutorial(group_name = left_arm_group_name)
    
    
    left_arm_group.move_group.set_max_velocity_scaling_factor(0.8)
    right_arm_group.move_group.set_max_velocity_scaling_factor(0.8)
    left_arm_group.move_group.set_max_acceleration_scaling_factor(0.8)
    right_arm_group.move_group.set_max_acceleration_scaling_factor(0.8)    
    
    
    with open(  right_arm_group_name + ".yaml", 'r') as file:
        right_j_configs = yaml.safe_load(file)
    with open(  left_arm_group_name + ".yaml", 'r') as file:
        left_j_configs = yaml.safe_load(file)       
    print(right_j_configs.keys(), left_j_configs.keys()) 
    right_js_idx = list(right_j_configs.keys())
    left_js_idx = list(left_j_configs.keys())
    right_js_idx.sort()
    left_js_idx.sort()
    assert(len(left_js_idx) != 0)
    assert(len(right_js_idx)!= 0)
    while(len(left_js_idx) != 0 and len(right_js_idx) != 0 ):
        
        if(len(right_js_idx) != 0):
            print("right arm configs left: ",  len(right_js_idx))
            right_arm_group.go_to_joint_goal(right_j_configs[right_js_idx.pop()])
            time.sleep(7.0)
            
        if(len(left_js_idx) != 0):
            print("left arm configs left: ",  len(left_js_idx))
            left_arm_group.go_to_joint_goal(left_j_configs[left_js_idx.pop()])
            time.sleep(7.0)             
         
    
    
    