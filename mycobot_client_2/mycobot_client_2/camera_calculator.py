from collections import deque
from dataclasses import dataclass
import threading
import struct

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
# import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

import matplotlib.pyplot as plt
import numpy as np
import numpy.typing as npt
import cv2

COLOR_CAMERA_TOPIC_NAME = "/camera/realsense2_camera_node/color/image_rect_raw"
COLOR_CAMERA_INFO_TOPIC_NAME = "/camera/realsense2_camera_node/color/image_rect_raw/camera_info"
DEPTH_CAMERA_TOPIC_NAME = "/camera/realsense2_camera_node/depth/image_rect_raw"
DEPTH_CAMERA_INFO_TOPIC_NAME = "/camera/realsense2_camera_node/depth/image_rect_raw/camera_info"
POINTCLOUD_TOPIC_NAME = "/camera/pointcloud"

@dataclass
class Images:
    id_num: int
    color: npt.NDArray[float]
    depth: npt.NDArray[float]
    xyz_rgb: npt.NDArray[np.float32]
    pointcloud_frame: str



class CameraCalculator(Node):
    def __init__(self):
        super().__init__('camera_calculator_node')
        self.br = CvBridge()
        self.color_sub = self.create_subscription(Image, COLOR_CAMERA_TOPIC_NAME, self.color_img_cb, 1)
        self.depth_sub = self.create_subscription(Image, DEPTH_CAMERA_TOPIC_NAME, self.depth_img_cb, 1)
        self.color_info_sub = self.create_subscription(CameraInfo, COLOR_CAMERA_INFO_TOPIC_NAME, self.color_img_info_cb, 1)
        self.depth_info_sub = self.create_subscription(CameraInfo, DEPTH_CAMERA_INFO_TOPIC_NAME, self.depth_img_info_cb, 1)
        self.pcd_publisher = self.create_publisher(PointCloud2, POINTCLOUD_TOPIC_NAME, 1)

        self.color_img_frame = None
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
        self.depth_img_cv = self.br.imgmsg_to_cv2(msg, "16UC1")
    
    def get_images(self):
        if self.color_img_cv is None or self.depth_img_cv is None or self.depth_processed_intrinsics is None:
            print("none")
            return None
        img_id = self.img_id_counter
        self.img_id_counter += 1

        xyz_rgb = self.get_3d_points(self.color_img_cv, self.depth_img_cv)
        pointcloud = self.points_to_pountcloud(xyz_rgb)
        self.pcd_publisher.publish(pointcloud)
        # get a color image and it's ID
        img = Images(img_id, self.color_img_cv, self.depth_img_cv, xyz_rgb, self.color_img_frame)
        return img

    def get_3d_points(self, color_img: npt.NDArray[float], depth_img: npt.NDArray[float]):

        if color_img.shape[0] != depth_img.shape[0] or color_img.shape[1] != depth_img.shape[1]:
            return None

        u = np.arange(0, color_img.shape[0] * color_img.shape[1], 1, dtype=np.float32)
        v = np.arange(0, color_img.shape[0] * color_img.shape[1], 1, dtype=np.float32)
        z = depth_img.flatten()

        fx = self.depth_processed_intrinsics[0, 0]
        fy = self.depth_processed_intrinsics[1, 1]
        cx = self.depth_processed_intrinsics[0, 2]
        cy = self.depth_processed_intrinsics[1, 2]

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        pixel_colors = color_img.reshape((x.shape[0], 3))

        # 1 channel, with the float in the channel reinterpreted as 3 single-byte values with ranges from 0 to 255. 0xff0000 is red, 0xff00 is green, 0xff is blue.

# In C++, int rgb = 0xff0000; float float_rgb = *reinterpret_cast<float*>(&rgb); 

# In Python,  float_rgb = struct.unpack('f', struct.pack('i', 0xff0000))[0]
        rgb = np.bitwise_or(np.left_shift(pixel_colors[:, 0], 4), np.bitwise_or(np.left_shift(pixel_colors[:, 1], 2), pixel_colors[:, 0]))
        rgb_float = np.array([struct.unpack('f', struct.pack('i', rgb[i])) for i in range(len(rgb))], dtype=np.float32)

        return np.concatenate((x[:, None], y[:, None], z[:, None], rgb_float), axis=1, dtype=np.float32)

    def points_to_pountcloud(self, xyz_rgb):

        fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
        fields += [PointField(name="rgb", offset=3*4, datatype=PointField.FLOAT32, count=1)]

        # The PointCloud2 message also has a header which specifies which 
        # coordinate frame it is represented in. 
        header =  Header(frame_id=self.color_img_frame)
        header.stamp = self.get_clock().now().to_msg()
        print(xyz_rgb.shape)
        print(xyz_rgb.dtype)

        # Data size (13025280 bytes) does not match width (407040) times height (1) times point_step (4). Dropping message.

        return PointCloud2(
            header=header,
            height=1, 
            width=xyz_rgb.shape[0],
            is_dense=False,
            is_bigendian=False,
            fields=fields,
            point_step=4 * 4, # Every point consists of 4 float32s, so 4 * 4 bytes.
            row_step=4 * xyz_rgb.shape[0],
            data=xyz_rgb.tobytes()
        )


    def display_img(self, img: Images):
        fig, axes = plt.subplots(2, 1, sharex=True, sharey=True)
        axes[0].imshow(img.color)
        depth_img = cv2.applyColorMap(cv2.convertScaleAbs(img.depth, alpha=0.03), cv2.COLORMAP_JET)
        axes[1].imshow(depth_img)
        plt.show()


def main(args=None):

    rclpy.init(args=args)

    camera_calculator = CameraCalculator()

    thread = threading.Thread(
    target=rclpy.spin, args=(camera_calculator, ), daemon=True)
    thread.start()

    rate = camera_calculator.create_rate(30)

    while rclpy.ok():
        print("getting image")
        img = camera_calculator.get_images()
        if img is None:
            continue
        # camera_calculator.display_img(img)
        rate.sleep()
        
    
    camera_calculator.stop()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_calculator.destroy_node()
    rclpy.shutdown()   


if __name__ == '__main__':
    main()