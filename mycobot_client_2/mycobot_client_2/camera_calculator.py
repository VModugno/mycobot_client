from collections import deque
from dataclasses import dataclass
import threading
import struct
import sys

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
# import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header

import matplotlib.pyplot as plt
import numpy as np
from numpy.linalg import eig as eigenValuesAndVectors
import numpy.typing as npt
import cv2


PITCH_DEGREES = 45
YAW_DEGREES = 45
TRANSLATION_Z = 0.210
TRANSLATION_Y = 0.180
DEGREES_TO_RADIAN = 1 * np.pi / 180

# cubes are 25mm a side
GREEN_CUBE_BOTTOM_LEFT_COORDS = np.array([0.1, -0.125, 0])
GREEN_CUBE_TOP_LEFT_COORDS = np.array([0.125, -0.125, 0])
GREEN_CUBE_BOTTOM_LEFT_COORDS_PIXEL = np.array([607, 233])
GREEN_CUBE_TOP_LEFT_COORDS_PIXEL = np.array([582, 218])

ORANGE_CUBE_BOTTOM_LEFT_COORDS = np.array([0.175, -0.125, 0])
ORANGE_CUBE_BOTTOM_LEFT_COORDS_PIXEL = np.array([532, 190])
ORANGE_CUBE_TOP_LEFT_COORDS = np.array([0.175, -0.125, 0])
ORANGE_CUBE_TOP_LEFT_COORDS_PIXEL = np.array([510, 178])

RED_CUBE_BOTTOM_LEFT_COORDS = np.array([0.175, 0.1, 0])
RED_CUBE_BOTTOM_LEFT_COORDS_PIXEL = np.array([317, 321])
RED_CUBE_TOP_LEFT_COORDS = np.array([0.175, 0.1, 0])
RED_CUBE_TOP_LEFT_COORDS_PIXEL = np.array([296, 301])

YELLOW_CUBE_BOTTOM_LEFT_COORDS = np.array([0.075, 0.1, 0])
YELLOW_CUBE_BOTTOM_LEFT_COORDS_PIXEL = np.array([431, 448])
YELLOW_CUBE_BOTTOM_RIGHT_COORDS = np.array([0.175, 0.1, 0])
YELLOW_CUBE_BOTTOM_RIGHT_COORDS_PIXEL = np.array([500, 454])


CUBE_WORLD_POINTS = np.array([GREEN_CUBE_BOTTOM_LEFT_COORDS, GREEN_CUBE_TOP_LEFT_COORDS, ORANGE_CUBE_BOTTOM_LEFT_COORDS,
                    ORANGE_CUBE_TOP_LEFT_COORDS, RED_CUBE_BOTTOM_LEFT_COORDS, RED_CUBE_TOP_LEFT_COORDS, YELLOW_CUBE_BOTTOM_LEFT_COORDS, YELLOW_CUBE_BOTTOM_RIGHT_COORDS])
CUBE_PIXEL_COORDS = np.array([GREEN_CUBE_BOTTOM_LEFT_COORDS_PIXEL, GREEN_CUBE_TOP_LEFT_COORDS_PIXEL, ORANGE_CUBE_BOTTOM_LEFT_COORDS_PIXEL, 
                ORANGE_CUBE_TOP_LEFT_COORDS_PIXEL, RED_CUBE_BOTTOM_LEFT_COORDS_PIXEL, RED_CUBE_TOP_LEFT_COORDS_PIXEL, YELLOW_CUBE_BOTTOM_LEFT_COORDS_PIXEL, YELLOW_CUBE_BOTTOM_RIGHT_COORDS_PIXEL])

# in cad, in global coordinate system X/Y/Z right handed like ros, we translated in Y, then in Z.
# then we rotated around z by 45 degrees, then we rotated around y by 45 degrees. This point, is in the middle of the realsense
# between the back and front (front has lenses)
# it is on the bottom of the realsense, when camera plug is coming out the left side of realsense.

# this frame has z pointing out of the camera, x pointing to the right in image plane
# and y pointing down in image plane
# we need to translate from this to the point we have in cad
# we can do this by taking xyz points in this frame and setting:
# z = -y
# x = z
# y = -x
# then translating to the point in cad
# then translating to the origin of the world
# from the datasheet, rom Centerline of ¼-20(1) To Left Imager: D405: 9 mm
# the centerline of the camera tripod is where our point in cad is. Left imager is depth frame. So let's take that and transform it back.
# from that campera tripod mount to the left imager is 21mm according to the datasheet upwards. So that tells us how to translate from the cad.
CAD_POINT_TO_CAMERA_FRAME_Z_OFFSET = 0.021
CAD_POINT_TO_CAMERA_FRAME_Y_OFFSET = 0.009
COLOR_CAMERA_FRAME_ID = "camera_color_optical_frame"
DEPTH_CAMERA_FRAME_ID = "camera_depth_optical_frame"
WORLD_FRAME_ID = "map"

Y_ROTATION = np.array([[np.cos(DEGREES_TO_RADIAN * PITCH_DEGREES), 0, np.sin(DEGREES_TO_RADIAN * PITCH_DEGREES)],
                        [0, 1, 0],
                        [-np.sin(DEGREES_TO_RADIAN * PITCH_DEGREES), 0, np.cos(DEGREES_TO_RADIAN * PITCH_DEGREES)]])
Z_ROTATION = np.array([[np.cos(DEGREES_TO_RADIAN * YAW_DEGREES), -np.sin(DEGREES_TO_RADIAN * YAW_DEGREES), 0],
                        [np.sin(DEGREES_TO_RADIAN * YAW_DEGREES), np.cos(DEGREES_TO_RADIAN * YAW_DEGREES), 0],
                        [0, 0, 1]])

COLOR_CAMERA_TOPIC_NAME = "/camera/realsense2_camera_node/color/image_rect_raw"
COLOR_CAMERA_INFO_TOPIC_NAME = "/camera/realsense2_camera_node/color/image_rect_raw/camera_info"
DEPTH_CAMERA_TOPIC_NAME = "/camera/realsense2_camera_node/depth/image_rect_raw"
DEPTH_CAMERA_INFO_TOPIC_NAME = "/camera/realsense2_camera_node/depth/image_rect_raw/camera_info"
POINTCLOUD_TOPIC_NAME = "/camera/pointcloud"

# d400 series have this set to 1mm by default
# https://github.com/IntelRealSense/librealsense/wiki/Projection-in-RealSense-SDK-2.0#depth-image-formats
DEPTH_SCALE = 0.001

WORLD_ROTATION_MAT = None
WORLD_TRANSFORM_MAT = None

def get_transform_mat(K):
    print("K")
    print(K)
    A = np.zeros((2 * CUBE_WORLD_POINTS.shape[0], 9))
    counter = 0
    for i in range(CUBE_WORLD_POINTS.shape[0]):
        xyz_coords = CUBE_WORLD_POINTS[i]
        pixel_coords = CUBE_PIXEL_COORDS[i]
        A[i* 2, 0] = -xyz_coords[0]
        A[i* 2, 1] = -xyz_coords[1]
        A[i* 2, 2] = -1
        A[i* 2, 6] = xyz_coords[0] * pixel_coords[0]
        A[i* 2, 7] = xyz_coords[1] * pixel_coords[0]
        A[i* 2, 8] = 1 * pixel_coords[0]

        A[i * 2 + 1, 3] = -xyz_coords[0]
        A[i * 2 + 1, 4] = -xyz_coords[1]
        A[i * 2 + 1, 5] = -1
        A[i * 2 + 1, 6] = xyz_coords[0] * pixel_coords[1]
        A[i * 2 + 1, 7] = xyz_coords[0] * pixel_coords[1]
        A[i * 2 + 1, 8] = 1 * pixel_coords[1]

    _, _, V = np.linalg.svd(A)
    H = V[-1].reshape(3, 3)

    KinvH = np.linalg.inv(K) @ H

    KinvH = KinvH / np.linalg.norm(KinvH[:, 0]) # Normalize to the first column

    r0 = KinvH[:, 0]
    r1 = KinvH[:, 1]
    r2 = np.cross(r0, r1)
    t = KinvH[:, 2]

    R = np.column_stack([r0, r1, r2])

    print("R")
    print(R)

    print("t")
    print(t)


    print(CUBE_WORLD_POINTS[-1])
    print(R @ CUBE_WORLD_POINTS[-1] + t)
    print(CUBE_PIXEL_COORDS[-1])
    return R, t

@dataclass
class Images:
    id_num: int
    color: npt.NDArray[float]
    depth: npt.NDArray[float]
    xyz_rgb: npt.NDArray[np.float32]
    pointcloud_frame: str
    u_v_mapping: npt.NDArray[float]


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
        self.depth_img_cv = self.br.imgmsg_to_cv2(msg, "16UC1")
    
    def translate_to_world_frame(self, xyz_rgb):
        # go from camera optical frame to camera link frame (a typical right handed system)
        # we can do this by taking xyz points in this frame and setting:
        # z = -y
        # x = z
        # y = -x
        x = np.copy(xyz_rgb[:, 0])
        y = np.copy(xyz_rgb[:, 1])
        z = np.copy(xyz_rgb[:, 2])
        xyz_rgb[:, 0] = z
        xyz_rgb[:, 1] = -x
        xyz_rgb[:, 2] = -y
        # now let's undo the pitch
        xyz_rgb[:, 0:3] = (Y_ROTATION.T @ xyz_rgb[:, 0:3].T).T
        # now let's undo the yaw
        xyz_rgb[:, 0:3] = (Z_ROTATION.T @ xyz_rgb[:, 0:3].T).T

        # now xyz rotation is aligned with world frame but we are translated from the camera link to the point we have in cad
        xyz_rgb[:, 1] = xyz_rgb[:, 1]  - CAD_POINT_TO_CAMERA_FRAME_Y_OFFSET
        xyz_rgb[:, 2] = xyz_rgb[:, 2] - CAD_POINT_TO_CAMERA_FRAME_Z_OFFSET

        # now the point we know in CAD is aligned, but now we need to go back to the origin of the robot
        xyz_rgb[:, 1] = xyz_rgb[:, 1] - TRANSLATION_Y
        xyz_rgb[:, 2] = xyz_rgb[:, 2] - TRANSLATION_Z
        return xyz_rgb
    
    def get_images(self):
        if self.color_img_cv is None or self.depth_img_cv is None or self.depth_processed_intrinsics is None:
            self.get_logger().error("color img was none or detph image was none or depth intrinsics was none")
            return None
        img_id = self.img_id_counter
        self.img_id_counter += 1
        
        # i think color camera and depth camera are aligned for this camera...
        if self.color_img_frame != COLOR_CAMERA_FRAME_ID or self.depth_img_frame != DEPTH_CAMERA_FRAME_ID:
            print(CAMERA_FRAME_ID)
            print(self.depth_img_frame )
            print(self.color_img_frame )
            return None

        xyz_rgb = self.get_3d_points(self.color_img_cv, self.depth_img_cv)
        xyz_rgb_world_frame = self.translate_to_world_frame(xyz_rgb)
        pointcloud = self.points_to_pountcloud(xyz_rgb_world_frame)
        self.pcd_publisher.publish(pointcloud)
        u_v_mapping_world = xyz_rgb_world_frame[:, 0:3].reshape((self.color_img_cv.shape[0], self.color_img_cv.shape[1], 3))
        u_v_mapping_cam = xyz_rgb[:, 0:3].reshape((self.color_img_cv.shape[0], self.color_img_cv.shape[1], 3))
        # get a color image and it's ID
        img = Images(img_id, self.color_img_cv, self.depth_img_cv, xyz_rgb, self.color_img_frame, u_v_mapping_world, u_v_mapping_cam)
        return img

    def get_3d_points(self, color_img: npt.NDArray[float], depth_img: npt.NDArray[float]):

        if color_img.shape[0] != depth_img.shape[0] or color_img.shape[1] != depth_img.shape[1]:
            return None
        print("color image shape")
        print(color_img.shape)
        ny, nx = (color_img.shape[1], color_img.shape[0])
        x = np.arange(0, nx, 1, dtype=np.float32)
        y = np.arange(0, ny, 1, dtype=np.float32)
        u, v = np.meshgrid(x, y)
        # print("u first 5 in row0")
        # print(u[0, :5])
        # print("u first 5 in row1")
        # print(u[1, :5])
        u = u.flatten().astype(np.float32)
        v = v.flatten().astype(np.float32)

        print(depth_img.flatten()[0:5])
        z = DEPTH_SCALE * depth_img.flatten()
        z = z.astype(np.float32)
        print(z[0:5])


        fx = self.depth_processed_intrinsics[0, 0]
        fy = self.depth_processed_intrinsics[1, 1]
        cx = self.depth_processed_intrinsics[0, 2]
        cy = self.depth_processed_intrinsics[1, 2]

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
            
            r_b = struct.unpack('I', struct.pack(pack_char + 'I',r))[0]
            g_b = struct.unpack('I', struct.pack(pack_char + 'I',g))[0]
            b_b = struct.unpack('I', struct.pack(pack_char + 'I',b))[0]
            b_final = (r_b << 16) | ((g_b << 8) | b_b)
            f_final = struct.unpack( pack_char + 'f', struct.pack('I',b_final))[0]
            packed_as_floats[i] = f_final
        self.get_logger().debug("checking bit  calcs")
        self.get_logger().debug(np.array_str(pixel_colors[0]))
        self.get_logger().debug(np.array_str(np.unpackbits(pixel_colors[0])))
        self.get_logger().debug(np.array_str(np.unpackbits(packed_as_floats[0:1].view(np.uint8))))

        return np.concatenate((x[:, None], y[:, None], z[:, None], packed_as_floats[:, None]), axis=1, dtype=np.float32)

    def points_to_pountcloud(self, xyz_rgb):

        fields = [PointField(name=n, offset=i*4, datatype=PointField.FLOAT32, count=1) for i, n in enumerate('xyz')]
        fields += [PointField(name="rgb", offset=3*4, datatype=PointField.FLOAT32, count=1)]

        # The PointCloud2 message also has a header which specifies which 
        # coordinate frame it is represented in. 
        header =  Header(frame_id=WORLD_FRAME_ID)
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
            point_step=4 * 4, # Every point consists of 4 float32s, so 4 * 4 bytes.
            row_step=4 * xyz_rgb.shape[0],
            data=xyz_rgb.tobytes()
        )
    
    def get_3d_points_from_pixel_point_on_color(self, img: Images, u, v):

        global WORLD_ROTATION_MAT, WORLD_TRANSFORM_MAT
        if WORLD_ROTATION_MAT is None:
            WORLD_ROTATION_MAT, WORLD_TRANSFORM_MAT = get_transform_mat(self.color_processed_intrinsics[:3, :3])
            WORLD_TRANSFORM_MAT = WORLD_TRANSFORM_MAT.reshape((3,1))

        print(img.depth.shape)
        z = DEPTH_SCALE * img.depth[v, u]

        fx = self.depth_processed_intrinsics[0, 0]
        fy = self.depth_processed_intrinsics[1, 1]
        cx = self.depth_processed_intrinsics[0, 2]
        cy = self.depth_processed_intrinsics[1, 2]

        x = (u - cx) * z / fx
        y = (v - cy) * z / fy

        point_in_cam_frame = np.array([[x, y, z]]).T # 3x1
        print(WORLD_TRANSFORM_MAT)
        print(point_in_cam_frame.shape)
        print(WORLD_TRANSFORM_MAT.shape)
        point_in_world_frame = WORLD_ROTATION_MAT.T @ (point_in_cam_frame - WORLD_TRANSFORM_MAT)

        return point_in_cam_frame, point_in_world_frame


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
        img = camera_calculator.get_images()
        if img is None:
            camera_calculator.get_logger().error("frame was None")
            continue
        cam_points, world_points = camera_calculator.get_3d_points_from_pixel_point_on_color(img, 532, 175)
        print(img.u_v_mapping[175, 532])
        # camera_calculator.get_logger().info("cam frame: " + np.array_str(cam_points))
        # camera_calculator.get_logger().info("world frame: " + np.array_str(world_points))
        camera_calculator.display_img(img)
        rate.sleep()
        
    
    camera_calculator.stop()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_calculator.destroy_node()
    rclpy.shutdown()   


if __name__ == '__main__':
    main()