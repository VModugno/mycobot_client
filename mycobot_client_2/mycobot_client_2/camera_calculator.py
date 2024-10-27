from dataclasses import dataclass
from datetime import datetime
import os
import threading
import time
from typing import Optional, Tuple
import struct
import sys

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CompressedImage, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import Point
from std_msgs.msg import Header

import matplotlib.pyplot as plt
import numpy as np
import numpy.typing as npt
import cv2

# this comes from the camera_extrinsics_via_procrustes script, ty Eddie for the help!
CAMERA_FRAME_TO_ROBOT_FRAME_EXTRINSICS = np.array([[-0.6905175,  -0.50300851,  0.51977689,  0.0183475],
                                                   [-0.72207053, 0.43722654, -
                                                       0.53614093, 0.17781414],
                                                   [0.0424232,  -0.74553027, -
                                                       0.6651202,  0.22065401],
                                                   [0.,        0.,    0.,     1.]], dtype=np.float32)

COLOR_CAMERA_FRAME_ID = "camera_color_optical_frame"
DEPTH_CAMERA_FRAME_ID = "camera_depth_optical_frame"
WORLD_FRAME_ID = "map"

COLOR_CAMERA_TOPIC_NAME = "/camera/realsense2_camera_node/color/image_rect_raw"
COLOR_CAMERA_COMPRESSED_TOPIC_NAME = "/camera/realsense2_camera_node/color/image_rect_raw/compressed"
COLOR_CAMERA_INFO_TOPIC_NAME = "/camera/realsense2_camera_node/color/image_rect_raw/camera_info"
DEPTH_CAMERA_TOPIC_NAME = "/camera/realsense2_camera_node/depth/image_rect_raw"
DEPTH_CAMERA_COMPRESSED_TOPIC_NAME = "/camera/realsense2_camera_node/depth/image_rect_raw/compressedDepth"
DEPTH_CAMERA_INFO_TOPIC_NAME = "/camera/realsense2_camera_node/depth/image_rect_raw/camera_info"
POINTCLOUD_TOPIC_NAME = "/camera/pointcloud"
OBJECT_FOUND_TOPIC = "/camera/object_found"

# d400 series have this set to 1mm by default, the D405 has 10cm by default, but the ros driver puts this back to 1mm.
# https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#depth-image-formats
DEPTH_SCALE = 0.001


@dataclass
class Images:
    """This is a datastructure to hold color/depth images and intrinsics, as well as the pointcloud.
    It's a chunky boy, don't hold too many in memory at once.
    """
    color: npt.NDArray[float]
    depth: npt.NDArray[float]
    intrinsics: npt.NDArray[float]
    xyz_rgb: npt.NDArray[np.float32]
    xyz_rgb_frame: str


class CameraCalculator(Node):
    """This is a class to run calculations on the camera images. Like making pointclouds from them and 
    getting the world coordinates of a pixel in the image. This intended to make it easier for students to write logic
    to find things in the image and move the robot arm.

    Args:
        Node (_type_): _description_
    """
    def __init__(self, img_out_dir: str = None, use_compressed: bool = True, publish_pointcloud: bool = False):
        """_summary_

        Args:
            img_out_dir (str, optional): we often don't want to display images and block the main thread, and with the python
                environment popular with our devs, cv2.imshow doesn't work. So we save images to a directory when
                trying to visualize them. This argument is the directory on the computer.. Defaults to None, which
                corresponds to ~/img_plots.
            use_compressed (bool, optional): whether to read from compressed topics. The RGB cam can be read at ~50HZ, the depth
                cam at 11HZ with this set to True, but this class makes no effort to time synchronize the images so this 
                exposes the risk that you will get a wrong image pair. Defaults to False.
            publish_pointcloud (bool, optional): whether to convert an image pair to a pointcloud, include this in the output,
                and publish it. This takes additional computer resources.
        """
        super().__init__('camera_calculator_node')
        self.br = CvBridge()
        if use_compressed:
            self.color_sub = self.create_subscription(
                CompressedImage, COLOR_CAMERA_COMPRESSED_TOPIC_NAME, self.color_img_compressed_cb, 1)
            self.depth_sub = self.create_subscription(
                CompressedImage, DEPTH_CAMERA_COMPRESSED_TOPIC_NAME, self.depth_img_compressed_cb, 1)
        else:
            self.color_sub = self.create_subscription(
                Image, COLOR_CAMERA_TOPIC_NAME, self.color_img_cb, 1)
            self.depth_sub = self.create_subscription(
                Image, DEPTH_CAMERA_TOPIC_NAME, self.depth_img_cb, 1)
        self.color_info_sub = self.create_subscription(
            CameraInfo, COLOR_CAMERA_INFO_TOPIC_NAME, self.color_img_info_cb, 1)
        self.depth_info_sub = self.create_subscription(
            CameraInfo, DEPTH_CAMERA_INFO_TOPIC_NAME, self.depth_img_info_cb, 1)
        self.publish_pointcloud = publish_pointcloud
        if self.publish_pointcloud:
            self.pcd_publisher = self.create_publisher(
                PointCloud2, POINTCLOUD_TOPIC_NAME, 1)
        self.object_publisher = self.create_publisher(Point, OBJECT_FOUND_TOPIC, 1)

        self.color_img_frame = None
        self.depth_img_frame = None
        self.color_img_cv = None
        self.depth_img_cv = None
        self.color_intrinsics = None
        self.depth_intrinsics = None
        self.get_logger().info("Camera to Global transform is: ")
        self.get_logger().info("\n" + np.array_str(CAMERA_FRAME_TO_ROBOT_FRAME_EXTRINSICS))

        if img_out_dir is None:
            img_out_dir = os.path.expanduser("~")
            img_out_dir = os.path.join(img_out_dir, "img_plots")
            os.makedirs(img_out_dir, exist_ok=True)
        self.img_out_dir = img_out_dir
        self.img_name_fmt = "%Y-%m-%d_%H-%M-%S.%f"
        self.get_logger().info(f"saving figures to {self.img_out_dir}")

    def depth_img_info_cb(self, msg):
        self.depth_intrinsics = np.array(msg.k).reshape((3, 3))

    def color_img_info_cb(self, msg):
        self.color_intrinsics = np.array(msg.k).reshape((3, 3))

    def color_img_cb(self, msg):
        self.color_img_frame = msg.header.frame_id
        self.color_img_cv = self.br.imgmsg_to_cv2(msg, "rgb8")

    def depth_img_cb(self, msg):
        self.depth_img_frame = msg.header.frame_id
        self.depth_img_cv = self.br.imgmsg_to_cv2(msg, "16UC1")

    def color_img_compressed_cb(self, msg):
        self.color_img_frame = msg.header.frame_id
        self.color_img_cv = self.br.compressed_imgmsg_to_cv2(msg, "rgb8")
    
    def depth_img_compressed_cb(self, msg):
        # https://github.com/ros-perception/vision_opencv/issues/206
        self.depth_img_frame = msg.header.frame_id
        str_msg = msg.data
        buf = np.ndarray(shape=(1, len(str_msg)),
                          dtype=np.uint8, buffer=msg.data)
        depth_header_size = 12
        buf = buf[:, depth_header_size:]
        self.depth_img_cv  = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)

    def translate_to_world_frame(self, xyz_rgb: npt.NDArray[np.float32]) -> npt.NDArray[np.float32]:
        """Helper function to translate a pointcloud with rgb data from camera to world frame.

        Args:
            xyz_rgb (npt.NDArray[np.float32]): nx4 array. the 4 is x, y, z, and a float containing r,g,b, compressed for rviz.

        Returns:
            _type_: npt.NDArray[np.float32]: nx4 array, in world coordinates.
        """
        new_pc = (CAMERA_FRAME_TO_ROBOT_FRAME_EXTRINSICS @ np.hstack(
            (xyz_rgb[:, 0:3], np.ones((xyz_rgb.shape[0], 1))), dtype=np.float32).T).T
        # set RGB back and drop the 1 helper
        new_pc[:, 3] = xyz_rgb[:, 3]
        return new_pc

    def get_images(self) -> Images:
        """Helper function to get an image pair and calculate the pointcloud. It will return None if not available, so check for that.

        Returns:
            Images: object with the image pair, intrinsics, and pointcloud. It's a chunky boy so don't keep too many in memory.
        """
        if self.color_img_cv is None or self.depth_img_cv is None or self.color_intrinsics is None:
            self.get_logger().error(
                "color img was none or detph image was none or depth intrinsics was none")
            self.get_logger().error(
                f"color img was {self.color_img_cv is None}, depth image was {self.depth_img_cv is None}, intrinsics was {self.color_intrinsics is None}")
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
        if self.publish_pointcloud:
            xyz_rgb_camera = self.get_3d_points(img.color, img.depth, intrinsics)
            xyz_rgb_world_frame = self.translate_to_world_frame(xyz_rgb_camera)
            img.xyz_rgb = xyz_rgb_world_frame
            img.xyz_rgb_frame = WORLD_FRAME_ID
            pointcloud = self.points_to_pountcloud(
                xyz_rgb_world_frame, WORLD_FRAME_ID)
            self.pcd_publisher.publish(pointcloud)

        return img

    def get_3d_points(self, color_img: npt.NDArray[float], depth_img: npt.NDArray[float], intrinsics: npt.NDArray) -> npt.NDArray[np.float32]:
        """Helper function to get a pointcloud from an image pair and intrinsics. In the Camera frame. It will then take the RGB element and 
        pack those 3 bytes into a 4 byte float in a way that rviz2 can read.

        Args:
            color_img (npt.NDArray[float]): _description_
            depth_img (npt.NDArray[float]): _description_
            intrinsics (npt.NDArray): _description_

        Returns:
            npt.NDArray[np.float32]: _description_
        """

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

    def points_to_pountcloud(self, xyz_rgb: npt.NDArray[np.float32], frame_id: str) -> PointCloud2:
        """Helper function to take in a numpy pointcloud nx4, where the 4 is x, y, z, and a float containing RGB in a format for rviz.
        It packs this into a PointCloud2 object that can be published to ROS.

        Args:
            xyz_rgb (npt.NDArray[np.float32]): numpy pointcloud
            frame_id (str): what frame to use for the pointcloud.

        Returns:
            PointCloud2: _description_
        """

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

    def get_3d_points_from_pixel_point_on_color(self, img: Images, u: int, v: int) -> Tuple[npt.NDArray[np.float32], npt.NDArray[np.float32]]:
        """Function to take image pair and intrinsics, and calculate the 3d location in world frame of a pixel. You could use a color mask or 
        contour stuff to get the center of an object in pixel coordinates, then pass it to this function to get the 3d location
        of the object in the world frame.

        Args:
            img (Images): image pair and intrinsics
            u (int): the u pixel coordinate. Note that in a numpy image the rows correspond to v, the columns to u.
            v (int): the v pixel coordinate. Note that in a numpy image the rows correspond to v, the columns to u.

        Returns:
            Tuple[npt.NDArray[np.float32], npt.NDArray[np.float32]]: the first array is points in the camera frame, second array is points in the world frame
        """

        depth = DEPTH_SCALE * img.depth[v, u]

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
        """Helper function to display an image pair and optionally to mark a point on the image with a cross.

        Args:
            img (Images): _description_
            markpoint_u_v (Optional[Tuple[int, int]], optional): _description_. Defaults to None.
        """
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
        img_name = datetime.now().strftime(self.img_name_fmt) + ".jpg"
        fig.savefig(os.path.join(self.img_out_dir, img_name))
        plt.close(fig)

    def display_image_pair(self, image_one: npt.NDArray, image_two: npt.NDArray, markpoint_u_v: Optional[Tuple[int, int]] = None):
        """Helper function to display an image pair and optionally to mark a point on the image with a cross.

        Args:
            img (Images): _description_
            markpoint_u_v (Optional[Tuple[int, int]], optional): _description_. Defaults to None.
        """
        fig, axes = plt.subplots(2, 1, sharex=True, sharey=True)
        axes[0].imshow(image_one)
        axes[1].imshow(image_two)
        axes[0].set_ylabel("v")
        axes[0].set_xlabel("u")
        axes[1].set_ylabel("v")
        axes[1].set_xlabel("u")
        if markpoint_u_v is not None:
            axes[0].plot(markpoint_u_v[0], markpoint_u_v[1], "r+")
            axes[1].plot(markpoint_u_v[0], markpoint_u_v[1], "r+")
        
        img_name = datetime.now().strftime(self.img_name_fmt) + ".jpg"
        fig.savefig(os.path.join(self.img_out_dir, img_name))
        plt.close(fig)
    
    def publish_object_found(self, xyz: npt.NDArray[np.float32]):
        point = Point()
        point.x = xyz[0]
        point.y = xyz[1]
        point.z = xyz[2]
        self.object_publisher.publish(point)


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

        for u in (100, 200, 300, 400, 500, 600, 700, 800):
            for v in (100, 200, 300, 400):
                point_of_interest_u_v = np.array((u, v))
                cam_points, world_points = camera_calculator.get_3d_points_from_pixel_point_on_color(
                    img, point_of_interest_u_v[0], point_of_interest_u_v[1])
                camera_calculator.get_logger().info(
                    "point_of_interest in u,v, then camera, then world points:")
                camera_calculator.get_logger().info(np.array_str(point_of_interest_u_v))
                camera_calculator.get_logger().info(np.array_str(cam_points))
                camera_calculator.get_logger().info(np.array_str(world_points))
                # camera_calculator.display_images(img, point_of_interest_u_v)
                rate.sleep()
        rate.sleep()


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_calculator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
