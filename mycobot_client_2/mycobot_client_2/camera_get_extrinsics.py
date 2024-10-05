from collections import deque
from dataclasses import dataclass
import time
import os
import threading
import struct
import sys

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import eig as eigenValuesAndVectors
import numpy.typing as npt
import cv2

# d400 series have this set to 1mm by default
# https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#depth-image-formats
DEPTH_SCALE_D405 = 0.001

CHESSBOARD_SIZE = 0.02
N_CORNERS_X = 6
N_CORNERS_Y = 9
CHESSBOARD_STARTING_POINT = np.array([0.095, 0.080])

COLOR_CAMERA_FRAME_ID = "camera_color_optical_frame"
DEPTH_CAMERA_FRAME_ID = "camera_depth_optical_frame"
WORLD_FRAME_ID = "map"

COLOR_CAMERA_TOPIC_NAME = "/camera/realsense2_camera_node/color/image_rect_raw"
COLOR_CAMERA_INFO_TOPIC_NAME = "/camera/realsense2_camera_node/color/image_rect_raw/camera_info"
DEPTH_CAMERA_TOPIC_NAME = "/camera/realsense2_camera_node/depth/image_rect_raw"
DEPTH_CAMERA_INFO_TOPIC_NAME = "/camera/realsense2_camera_node/depth/image_rect_raw/camera_info"

@dataclass
class Images:
    id_num: int
    color: npt.NDArray[float]
    depth: npt.NDArray[float]


class CameraCalculator(Node):
    def __init__(self):
        super().__init__('camera_extrinsic_calculator')
        self.br = CvBridge()
        self.color_sub = self.create_subscription(Image, COLOR_CAMERA_TOPIC_NAME, self.color_img_cb, 1)
        self.depth_sub = self.create_subscription(Image, DEPTH_CAMERA_TOPIC_NAME, self.depth_img_cb, 1)
        self.color_info_sub = self.create_subscription(CameraInfo, COLOR_CAMERA_INFO_TOPIC_NAME, self.color_img_info_cb, 1)
        self.depth_info_sub = self.create_subscription(CameraInfo, DEPTH_CAMERA_INFO_TOPIC_NAME, self.depth_img_info_cb, 1)
        
        self.color_img_frame = None
        self.depth_img_frame = None
        self.color_img_cv = None
        self.depth_img_cv = None
        self.color_processed_intrinsics = None
        self.depth_processed_intrinsics = None
        self.img_id_counter = 0
    

    def depth_img_info_cb(self, msg):
        #  the rectified image is given by:
        #  [u v w]' = P * [X Y Z 1]'
        #         x = u / w
        #         y = v / w
        #  This holds for both images of a stereo pair.

        self.depth_processed_intrinsics = np.array(msg.p).reshape((3, 4))

    def color_img_info_cb(self, msg):
        #  the rectified image is given by:
        #  [u v w]' = P * [X Y Z 1]'
        #         x = u / w
        #         y = v / w
        #  This holds for both images of a stereo pair.

        self.color_processed_intrinsics = np.array(msg.p).reshape((3, 4))
    
    def color_img_cb(self, msg):
        self.color_img_frame = msg.header.frame_id
        self.color_img_cv = self.br.imgmsg_to_cv2(msg, "rgb8")
    
    def depth_img_cb(self, msg):
        self.depth_img_frame = msg.header.frame_id
        print(msg.height)
        print(msg.width)
        print(msg.encoding)
        print(msg.is_bigendian)
        print(msg.step)
        print(msg.data[0:50])
        self.depth_img_cv = self.br.imgmsg_to_cv2(msg, "16UC1")
        print(self.depth_img_cv[self.depth_img_cv.shape[1]//2, self.depth_img_cv.shape[0]//2])
    
    def get_images(self):
        if self.color_img_cv is None or self.depth_img_cv is None or self.depth_processed_intrinsics is None:
            self.get_logger().error("color img was none or detph image was none or depth intrinsics was none")
            return None
        img_id = self.img_id_counter
        self.img_id_counter += 1
        
        # i think color camera and depth camera are aligned for this parameter set...
        if self.color_img_frame != COLOR_CAMERA_FRAME_ID or self.depth_img_frame != DEPTH_CAMERA_FRAME_ID:
            self.get_logger().error(CAMERA_FRAME_ID)
            self.get_logger().error(self.depth_img_frame )
            self.get_logger().error(self.color_img_frame )
            return None

        img = Images(self.img_id_counter, self.color_img_cv, self.depth_img_cv)
        return img

    def calc(self):
        while rclpy.ok():
            img = self.get_images()
            if img is None or self.depth_processed_intrinsics is None:
                self.get_logger().error("frame or depth intrinsics was None")
                time.sleep(0.1)
                continue
            break
        file_dir = "/home/mz/mycobot_client/docs/raw_data"
        color_img_name = "chessboard_color_image.jpg"
        color_img_name_npy = "chessboard_color_image.npy"
        depth_img_name = "chessboard_depth_image.jpg"
        depth_img_npy = "chessboard_depth_image.npy"
        intrinsics_name = "depth_intrinsics.npy"
        with open(os.path.join(file_dir, color_img_name_npy), "wb") as file_handle:
            np.save(file_handle, img.color)
        with open(os.path.join(file_dir, depth_img_npy), "wb") as file_handle:
            np.save(file_handle, img.depth)
        cv2.imwrite(os.path.join(file_dir, color_img_name), img.color)
        cv2.imwrite(os.path.join(file_dir, depth_img_name), img.depth)
        with open(os.path.join(file_dir, intrinsics_name), "wb") as file_handle:
            np.save(file_handle, self.depth_processed_intrinsics)

        K = np.copy(self.depth_processed_intrinsics[:3, :3])
        fig = plt.figure()
        ax = plt.subplot()
        ax.imshow(img.color, cmap="gray")
        ax.set_ylabel("v")
        ax.set_xlabel("u")
        # get checkerboard
        ret, corners = cv2.findChessboardCorners(img.color, (N_CORNERS_X, N_CORNERS_Y))
        corners = corners.squeeze()
        self.get_logger().info("corners")
        self.get_logger().info(np.array_str(corners))

        ax.plot(corners[:, 0], corners[:, 1], "r+")
        for i in range(corners.shape[0]):
            u, v = corners[i, :]
            ax.text(u, v, f"{i}", color="b")
        plt.show()

        # the opencv find chess board seems to start in bottom left, go forward in x, then at end of x  row
        # go to next
        Xg = np.zeros((N_CORNERS_X * N_CORNERS_Y,))
        Yg = np.zeros((N_CORNERS_X * N_CORNERS_Y,))
        for i in range(N_CORNERS_Y):
            for j in range(N_CORNERS_X):
                x_coord = CHESSBOARD_STARTING_POINT[0] + j * CHESSBOARD_SIZE
                y_coord = CHESSBOARD_STARTING_POINT[1] - i * CHESSBOARD_SIZE
                idx = N_CORNERS_X * i + j
                Xg[idx] = x_coord
                Yg[idx] = y_coord

        self.get_logger().info(np.array_str(np.hstack((Xg[:, None], Yg[:, None]))))


        P = np.hstack((Xg[:, None], Yg[:, None], np.ones_like(Xg)[:, None]))
        print(P.shape)

        M_u = np.hstack((-P, np.zeros_like(P), corners[:, 0].reshape(-1, 1) * P))
        M_v = np.hstack((np.zeros_like(P), -P, corners[:, 1].reshape(-1, 1) * P))
        M = np.vstack((M_u, M_v))
        _, _, V = np.linalg.svd(M)
        H = V[-1].reshape(3, 3)

        KinvH = np.linalg.inv(K) @ H

        KinvH = KinvH / np.linalg.norm(KinvH[:, 0]) # Normalize to the first column

        r0 = KinvH[:, 0]
        r1 = KinvH[:, 1]
        r2 = np.cross(r0, r1)
        t = KinvH[:, 2]

        R = np.column_stack([r0, r1, r2])

        print("R: ", R, "\n")
        print("t: ", t)
        # print(P)

        p1_global = np.array((Xg[0], Yg[0], 0))
        print("global")
        print(p1_global)
        p1_u_v = np.array((corners[0, 0], corners[0, 1]))
        print("u, v")
        print(p1_u_v)

        print(img.depth.shape)
        depth = img.depth[int(p1_u_v[1]), int(p1_u_v[0])]
        depth = depth * DEPTH_SCALE_D405
        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[2, 2]

        x = (p1_u_v[0] - cx) * depth / fx
        y = (p1_u_v[1] - cy) * depth / fy
        z = depth

        p1_xyz_camera = np.array([x, y, z])
        p1_xyz_recreated_camera = R @ p1_global + t

        print("xyz cam")
        print(p1_xyz_camera)
        print("camera recreated")
        print(p1_xyz_recreated_camera)

        global_recreated = R.T @ (p1_xyz_camera - t)
        print("global recreated from cam coords")
        print(global_recreated)

        print("global recreated from recreated global")
        print(R.T @ (global_recreated - t))

        print("cam to global")
        print(R @ p1_xyz_camera + t)
    
    
    

def main(args=None):

    rclpy.init(args=args)

    camera_calculator = CameraCalculator()

    thread = threading.Thread(
    target=rclpy.spin, args=(camera_calculator, ), daemon=True)
    thread.start()

    calc = camera_calculator.calc()
        
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_calculator.destroy_node()
    rclpy.shutdown()   


if __name__ == '__main__':
    main()