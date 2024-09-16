import numpy as np
import matplotlib.pyplot as plt
import cv2
from robot_arm_algos.src.tags_detection.aruco_board import ArucoBoard, ArucoBoardData
from robot_arm_algos.src.camera.camera import Camera
from robot_arm_algos.src.logger import logger
# from robot_arm_algos.src.robot_camera_calibration.data_collector.arucoboard_data_collector import ArucoBoardDataCollector

def main():
    camera_matrix = np.array([[909.34179688, 0.0, 643.8762207 ],
                            [0.0, 908.06414795, 348.60513306],
                            [0.0, 0.0, 1.0]])
    dist_coeffs = np.array([0.0,0.0,0.0,0.0])                            
    camera = Camera(camera_matrix, dist_coeffs)
    aruco_board_data = ArucoBoardData(dictionary="DICT_6X6_1000",
                                    marker_length = 0.025,
                                    marker_separation = 0.005,
                                    n_rows = 5,
                                    n_cols = 7)
    aruco_board = ArucoBoard(aruco_board_data)
    color_image = cv2.imread("tests/data/aruco_board_test.png")
    corners, ids, _  = aruco_board.detect_markers(color_image)
    image_gray =  cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)    
    aruco_board.refine_corners(image_gray, corners)
    obj_points, img_points = aruco_board.get_matched_object_points(corners, ids)
    rvec, tvec = aruco_board.estimate_pose(obj_points, img_points, camera)
    reproj_error, error_variance = aruco_board.compute_reprojection_error(rvec,
                                                        tvec,
                                                        obj_points,
                                                        img_points,
                                                        camera)
    logger.info(f"reprojection error = {reproj_error}, error_variance = {error_variance}")
    aruco_board.draw_detections(color_image, corners)
    aruco_board.draw_estimated_frame(color_image, camera, rvec, tvec)
    plt.imshow(color_image)
    plt.show(block=False)
    plt.pause(4.0)
    plt.close()
    assert(aruco_board.fiducial_type == "aruco_board")


if __name__ == "__main__":
    main()

