rosbag record /joint_states     \
            /tf                 \
            /tf_static          \
            /zed2i/zed_node/right_raw/image_raw_color/compressed \
            /zed2i/zed_node/right_raw/camera_info \
            /zed2i/zed_node/left_raw/image_raw_color/compressed \
            /zed2i/zed_node/left_raw/camera_info \
            /right_arm_camera/color/image_raw \
            /right_arm_camera/color/camera_info\
            /left_arm_camera/color/image_raw \
            /left_arm_camera/color/camera_info \
            /ouster/points -o jango_1.bag
 


#cp src/tor42_ridgeback/tor42_ridgeback_description/urdf/tor42_ridgeback.urdf /tmp/description.urdf 
