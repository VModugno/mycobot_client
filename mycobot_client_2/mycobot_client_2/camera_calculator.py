from collections import deque
from dataclasses import dataclass
import threading
import time
from typing import Optional, Tuple
import struct
import sys

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header

import matplotlib.pyplot as plt
import numpy as np
import numpy.typing as npt
import cv2

CAMERA_FRAME_TO_ROBOT_FRAME_EXTRINSICS = np.array([[-0.66084513, -0.26978085, 0.70035849, 0.0030016],
                                                   [-0.74858188, 0.16987259, -
                                                       0.64091222, 0.18775045],
                                                   [0.05393414, -0.94781939, -
                                                       0.31421252, 0.28052536],
                                                   [0.,       0.,     0.,       1.]], dtype=np.float32)

COLOR_CAMERA_FRAME_ID = "camera_color_optical_frame"
DEPTH_CAMERA_FRAME_ID = "camera_depth_optical_frame"
WORLD_FRAME_ID = "map"

COLOR_CAMERA_TOPIC_NAME = "/camera/realsense2_camera_node/color/image_rect_raw"
COLOR_CAMERA_INFO_TOPIC_NAME = "/camera/realsense2_camera_node/color/image_rect_raw/camera_info"
DEPTH_CAMERA_TOPIC_NAME = "/camera/realsense2_camera_node/depth/image_rect_raw"
DEPTH_CAMERA_INFO_TOPIC_NAME = "/camera/realsense2_camera_node/depth/image_rect_raw/camera_info"
POINTCLOUD_TOPIC_NAME = "/camera/pointcloud"

# d400 series have this set to 1mm by default, the D405 has 10cm by default, but the ros driver puts this back to 1mm.
# https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#depth-image-formats
DEPTH_SCALE = 0.001


@dataclass
class Images:
    color: npt.NDArray[float]
    depth: npt.NDArray[float]
    intrinsics: npt.NDArray[float]
    xyz_rgb: npt.NDArray[np.float32]
    xyz_rgb_frame: str


class CameraCalculator(Node):
    def __init__(self):
        super().__init__('camera_calculator_node')
        self.br = CvBridge()
        self.color_sub = self.create_subscription(
            Image, COLOR_CAMERA_TOPIC_NAME, self.color_img_cb, 1)
        self.depth_sub = self.create_subscription(
            Image, DEPTH_CAMERA_TOPIC_NAME, self.depth_img_cb, 1)
        self.color_info_sub = self.create_subscription(
            CameraInfo, COLOR_CAMERA_INFO_TOPIC_NAME, self.color_img_info_cb, 1)
        self.depth_info_sub = self.create_subscription(
            CameraInfo, DEPTH_CAMERA_INFO_TOPIC_NAME, self.depth_img_info_cb, 1)
        self.pcd_publisher = self.create_publisher(
            PointCloud2, POINTCLOUD_TOPIC_NAME, 1)

        self.color_img_frame = None
        self.depth_img_frame = None
        self.color_img_cv = None
        self.depth_img_cv = None
        self.color_intrinsics = None
        self.depth_intrinsics = None
        self.viewing_image = False

    def depth_img_info_cb(self, msg):
        #  the rectified image is given by:
        #  [u v w]' = P * [X Y Z 1]'
        #         x = u / w
        #         y = v / w
        #  This holds for both images of a stereo pair.
        if self.viewing_image:
            return

        self.depth_intrinsics = np.array(msg.k).reshape((3, 3))

    def color_img_info_cb(self, msg):
        #  the rectified image is given by:
        #  [u v w]' = P * [X Y Z 1]'
        #         x = u / w
        #         y = v / w
        #  This holds for both images of a stereo pair.
        if self.viewing_image:
            return
        self.color_intrinsics = np.array(msg.k).reshape((3, 3))

    def color_img_cb(self, msg):
        if self.viewing_image:
            return
        self.color_img_frame = msg.header.frame_id
        self.color_img_cv = self.br.imgmsg_to_cv2(msg, "rgb8")

    def depth_img_cb(self, msg):
        if self.viewing_image:
            return
        self.depth_img_frame = msg.header.frame_id
        self.depth_img_cv = self.br.imgmsg_to_cv2(msg, "16UC1")

    def translate_to_world_frame(self, xyz_rgb):
        new_pc = (CAMERA_FRAME_TO_ROBOT_FRAME_EXTRINSICS @ np.hstack(
            (xyz_rgb[:, 0:3], np.ones((xyz_rgb.shape[0], 1))), dtype=np.float32).T).T
        # set RGB back and drop the 1 helper
        new_pc[:, 3] = xyz_rgb[:, 3]
        return new_pc

    def get_images(self):
        if self.color_img_cv is None or self.depth_img_cv is None or self.color_intrinsics is None:
            self.get_logger().error(
                "color img was none or detph image was none or depth intrinsics was none")
            return None

        # i think color camera and depth camera are aligned for this camera...
        if self.color_img_frame != COLOR_CAMERA_FRAME_ID or self.depth_img_frame != DEPTH_CAMERA_FRAME_ID:
            self.get_logger().error("frame ids were not as expected.")
            self.get_logger().error(COLOR_CAMERA_FRAME_ID)
            self.get_logger().error(DEPTH_CAMERA_FRAME_ID)
            self.get_logger().error(self.depth_img_frame)
            self.get_logger().error(self.color_img_frame)
            return None

        intrinsics = self.color_intrinsics
        img = Images(self.color_img_cv, self.depth_img_cv,
                     intrinsics, None, None)

        xyz_rgb_camera = self.get_3d_points(img.color, img.depth, intrinsics)
        xyz_rgb_world_frame = self.translate_to_world_frame(xyz_rgb_camera)
        img.xyz_rgb = xyz_rgb_world_frame
        img.xyz_rgb_frame = WORLD_FRAME_ID
        pointcloud = self.points_to_pountcloud(
            xyz_rgb_world_frame, WORLD_FRAME_ID)
        self.pcd_publisher.publish(pointcloud)

        return img

    def get_3d_points(self, color_img: npt.NDArray[float], depth_img: npt.NDArray[float], intrinsics: npt.NDArray):

        if color_img.shape[0] != depth_img.shape[0] or color_img.shape[1] != depth_img.shape[1]:
            return None
        ny, nx = (color_img.shape[1], color_img.shape[0])
        x = np.arange(0, nx, 1, dtype=np.float32)
        y = np.arange(0, ny, 1, dtype=np.float32)
        u, v = np.meshgrid(x, y)
        u = u.flatten().astype(np.float32)
        v = v.flatten().astype(np.float32)

        z = DEPTH_SCALE * depth_img.flatten()
        z = z.astype(np.float32)

        fx = intrinsics[0, 0]
        fy = intrinsics[1, 1]
        cx = intrinsics[0, 2]
        cy = intrinsics[1, 2]

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        pixel_colors = color_img.reshape((x.shape[0], 3))

        # for the colors of the pointcloud i want one float, where it's 4 bytes, and i have 3 bytes from the 8 bit integers
        # 0x 0 R G B
        # this is how rviz2 will work it out

        packed_as_floats = np.zeros(pixel_colors.shape[0], dtype=np.float32)
        pack_char = ""

        # there is probably a way to do this without a for loop, but i am tired
        for i in range(0, pixel_colors.shape[0]):
            r = pixel_colors[i, 0]
            g = pixel_colors[i, 1]
            b = pixel_colors[i, 2]

            r_b = struct.unpack('I', struct.pack(pack_char + 'I', r))[0]
            g_b = struct.unpack('I', struct.pack(pack_char + 'I', g))[0]
            b_b = struct.unpack('I', struct.pack(pack_char + 'I', b))[0]
            b_final = (r_b << 16) | ((g_b << 8) | b_b)
            f_final = struct.unpack(
                pack_char + 'f', struct.pack('I', b_final))[0]
            packed_as_floats[i] = f_final
        self.get_logger().debug("checking bit  calcs")
        self.get_logger().debug(np.array_str(pixel_colors[0]))
        self.get_logger().debug(np.array_str(np.unpackbits(pixel_colors[0])))
        self.get_logger().debug(np.array_str(
            np.unpackbits(packed_as_floats[0:1].view(np.uint8))))
        return np.concatenate((x[:, None], y[:, None], z[:, None], packed_as_floats[:, None]), axis=1, dtype=np.float32)

    def points_to_pountcloud(self, xyz_rgb, frame_id: str):

        fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate('xyz')]
        fields += [PointField(name="rgb", offset=3*4,
                              datatype=PointField.FLOAT32, count=1)]

        # The PointCloud2 message also has a header which specifies which
        # coordinate frame it is represented in.
        header = Header(frame_id=frame_id)
        header.stamp = self.get_clock().now().to_msg()

        endianness = sys.byteorder
        big_endian = endianness == "big"
        print(xyz_rgb.shape)
        print(xyz_rgb.dtype)

        return PointCloud2(
            header=header,
            height=1,
            width=xyz_rgb.shape[0],
            is_dense=False,
            is_bigendian=big_endian,
            fields=fields,
            # Every point consists of 4 float32s, so 4 * 4 bytes.
            point_step=4 * 4,
            row_step=4 * xyz_rgb.shape[0],
            data=xyz_rgb.tobytes()
        )

    def get_3d_points_from_pixel_point_on_color(self, img: Images, u: int, v: int):

        depth = DEPTH_SCALE * img.depth[v, u]

        print(img.intrinsics)

        fx = img.intrinsics[0, 0]
        fy = img.intrinsics[1, 1]
        cx = img.intrinsics[0, 2]
        cy = img.intrinsics[1, 2]

        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth

        point_in_cam_frame = np.array([[x, y, z]]).reshape((3, 1))
        point_in_global_frame = CAMERA_FRAME_TO_ROBOT_FRAME_EXTRINSICS @ np.vstack(
            (point_in_cam_frame, np.ones((1, 1))))
        point_in_global_frame = point_in_global_frame[:3, :]

        return point_in_cam_frame, point_in_global_frame

    def display_images(self, img: Images, markpoint_u_v: Optional[Tuple[int, int]] = None):
        self.viewing_image = True
        fig, axes = plt.subplots(2, 1, sharex=True, sharey=True)
        axes[0].imshow(img.color)
        depth_img = cv2.applyColorMap(cv2.convertScaleAbs(
            img.depth, alpha=0.03), cv2.COLORMAP_JET)
        axes[1].imshow(depth_img)
        axes[0].set_ylabel("v")
        axes[0].set_xlabel("u")
        axes[1].set_ylabel("v")
        axes[1].set_xlabel("u")
        if markpoint_u_v is not None:
            axes[0].plot(markpoint_u_v[0], markpoint_u_v[1], "r+")
            axes[1].plot(markpoint_u_v[0], markpoint_u_v[1], "r+")
        plt.show()
        self.viewing_image = False


def main(args=None):

    rclpy.init(args=args)

    camera_calculator = CameraCalculator()

    thread = threading.Thread(
        target=rclpy.spin, args=(camera_calculator, ), daemon=True)
    thread.start()

    rate = camera_calculator.create_rate(30)

    while rclpy.ok():
        img = camera_calculator.get_images()
        if img is None:
            camera_calculator.get_logger().error("frame was None")
            time.sleep(0.1)
            continue
        break
    for u in (100, 200, 300, 400, 500, 600, 700, 800):
        for v in (100, 200, 300, 400):
            # point_of_interest = (img.color.shape[0]//2, img.color.shape[1]//2)
            point_of_interest_u_v = np.array((u, v))
            cam_points, world_points = camera_calculator.get_3d_points_from_pixel_point_on_color(
                img, point_of_interest_u_v[0], point_of_interest_u_v[1])
            camera_calculator.get_logger().info(
                "point_of_interest in u,v, then camera, then world points:")
            camera_calculator.get_logger().info(np.array_str(point_of_interest_u_v))
            camera_calculator.get_logger().info(np.array_str(cam_points))
            camera_calculator.get_logger().info(np.array_str(world_points))
            camera_calculator.display_images(img, point_of_interest_u_v)
            rate.sleep()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_calculator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
