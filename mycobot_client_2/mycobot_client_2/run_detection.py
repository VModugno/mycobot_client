import threading
import time

import rclpy

import matplotlib.pyplot as plt
import numpy as np
import cv2

from mycobot_client_2.camera_calculator import CameraCalculator



def main(args=None):

    rclpy.init(args=args)

    camera_calculator = CameraCalculator()

    thread = threading.Thread(
        target=rclpy.spin, args=(camera_calculator, ), daemon=True)
    thread.start()

    rate = camera_calculator.create_rate(1)

    while rclpy.ok():
        img = camera_calculator.get_images()
        if img is None:
            camera_calculator.get_logger().error("frame was None")
            time.sleep(0.1)
            continue

        # TODO: add logic to detect cubes, perhaps by HSV color and finding contours, and publish the coordinates via camera_calculator.publish_object_found
        out_img = cv2.cvtColor(img.color, cv2.COLOR_RGB2HSV)
        found_obj = False
        # setting u, v, to center of screen for example purposes
        u, v = img.color.shape[1]//2, img.color.shape[0]//2
        camera_frame_coordinates, world_frame_coordinates = camera_calculator.get_3d_points_from_pixel_point_on_color(img, u, v)
        if found_obj:
            world_coords = np.array((0.2, 0.2, 0))
            camera_calculator.publish_object_found(world_coords.reshape((3,)))

        camera_calculator.display_image_pair(img.color, out_img)

        rate.sleep()


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_calculator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
