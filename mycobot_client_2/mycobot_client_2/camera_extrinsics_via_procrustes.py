import os

import cv2
from sksurgerycore.algorithms.procrustes import orthogonal_procrustes
import matplotlib.pyplot as plt
import numpy as np


class Intrinsics:
    def __init__(self, fx, fy, ppx, ppy):
        self.fx = fx
        self.fy = fy
        self.ppx = ppx  # c_x
        self.ppy = ppy  # c_y

# Instantiate intrinsics with example values

def deproject_pixel_to_point(intrinsics, pixel, depth):
    """
    Deprojects a 2D pixel coordinate into a 3D point using camera intrinsics.

    Parameters:
    - intrinsics: An object containing camera intrinsic parameters (fx, fy, ppx, ppy).
    - pixel: A list or tuple with the pixel coordinates [u, v].
    - depth: The depth value (Z coordinate) at the pixel.

    Returns:
    - A list [X, Y, Z] representing the 3D point in camera coordinates.
    """
    u, v = pixel
    fx = intrinsics.fx
    fy = intrinsics.fy
    cx = intrinsics.ppx
    cy = intrinsics.ppy

    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    Z = depth

    return [X, Y, Z]


def main():

    file_dir = "/home/mz/mycobot_client/docs/raw_data"
    color_img_name = "chessboard_color_image.jpg"
    color_img_name_npy = "chessboard_color_image.npy"
    depth_img_name = "chessboard_depth_image.jpg"
    depth_img_npy = "chessboard_depth_image.npy"
    intrinsics_name = "depth_intrinsics.npy"

    DEPTH_SCALE = 0.001

    CHESSBOARD_SIZE = 0.02
    N_CORNERS_X = 6
    N_CORNERS_Y = 9
    CHESSBOARD_STARTING_POINT = np.array([0.095, 0.080])

    full_intrinsics = None

    with open(os.path.join(file_dir, intrinsics_name), 'rb') as f:
        full_intrinsics = np.load(f)

    with open(os.path.join(file_dir, color_img_name_npy), 'rb') as f:
        color_img = np.load(f)
    
    with open(os.path.join(file_dir, depth_img_npy), 'rb') as f:
        depth_img = np.load(f)

    fig, axes = plt.subplots(2, 1, sharex=True, sharey=True)
    axes[0].imshow(color_img)
    depth_img_dis = cv2.applyColorMap(cv2.convertScaleAbs(depth_img, alpha=0.03), cv2.COLORMAP_JET)
    axes[1].imshow(depth_img_dis)
    plt.show()

    # color_img = cv2.imread(os.path.join(file_dir, color_img_name))
    # depth_img = cv2.imread(os.path.join(file_dir, depth_img_name))
    fx = full_intrinsics[0, 0]
    fy = full_intrinsics[1, 1]
    cx = full_intrinsics[0, 2]
    cy = full_intrinsics[1, 2]
    depth_intrinsics = Intrinsics(fx, fy, cx, cy)

    print("full intrinsics (P)")
    print(full_intrinsics)

    fig = plt.figure()
    ax = plt.subplot()
    ax.imshow(color_img, cmap="gray")
    ax.set_ylabel("v")
    ax.set_xlabel("u")
    # get checkerboard
    ret, corners = cv2.findChessboardCorners(color_img, (N_CORNERS_X, N_CORNERS_Y))
    if not ret:
        raise RuntimeError
    corners = corners.squeeze()
    print("corners")
    print(np.array_str(corners))

    ax.plot(corners[:, 0], corners[:, 1], "r+")
    # for i in range(corners.shape[0]):
    #     u, v = corners[i, :]
    #     ax.text(u, v, f"{i}", color="b")
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

    z_height = 0.0
    points_global = np.hstack((Xg[:, None], Yg[:, None], np.ones((Xg.shape[0], 1)) * z_height))

    print("global points")
    print(np.array_str(points_global))

    points_camera = np.zeros((corners.shape[0], 3))

    for i in range(len(corners)):
        u, v = corners[i]
        print(depth_img.shape)
        unscaled_depth = depth_img[int(v), int(u)] 
        print(unscaled_depth)
        depth = depth_img[int(v), int(u)] * DEPTH_SCALE
        res = deproject_pixel_to_point(depth_intrinsics, [u, v], depth)
        points_camera[i, :] = res

    print("camera points")
    print(np.array_str(points_camera))

    R, t, FRE = orthogonal_procrustes(points_global, points_camera)
    print("rotation")
    print(R)
    print("translation")
    print(t)
    print("Fiducial Residual Error")
    print(FRE)

    transformation = np.zeros((4,4))
    transformation[:3,:3] = R
    transformation[0:3, 3] = t[:, 0]
    transformation[3, 3] = 1

    print("transform matrix")
    print(transformation)

    for chess_point_index in range(corners.shape[0]):
        u, v = corners[chess_point_index]
        point_cam = points_camera[chess_point_index]
        point_global = points_global[chess_point_index]
        recreated_global = transformation @ np.vstack((point_cam.reshape((3,1)), np.ones((1,1))))
        recreated_camera = np.linalg.inv(transformation) @ np.vstack((point_global.reshape((3,1)), np.ones((1,1))))
        print(f"chess corner {chess_point_index}")
        print("u, v")
        print(u, v)
        print("camera")
        print(point_cam)
        print("global")
        print(point_global)
        print("recreated global")
        print(recreated_global)
        print("recreated camera")
        print(recreated_camera)

    for chess_point_index in range(corners.shape[0]):
        point_cam = points_camera[chess_point_index]
        print(point_cam[2])


    cam_point_chess = np.array([[-0.04158675],
 [ 0.06369323],
 [ 0.257     ],
 [1]])
    global_recreated_via_transform = transformation @ cam_point_chess
    print("global_cam_point_chess")
    print(global_recreated_via_transform)

    another_point_on_table = np.array([[-0.13714086],
 [ 0.03687973],
 [ 0.277     ],
 [1]])
    global_recreated_via_transform = transformation @ another_point_on_table
    print("global_cam_point_chess 2")
    print(global_recreated_via_transform)

    u, v = (color_img.shape[1]//2 , color_img.shape[0]//2)
    cam_coords = deproject_pixel_to_point(depth_intrinsics, (u,v), DEPTH_SCALE * depth_img[v, u])
    print("middle of img")
    cam_coords.append(1)
    cam_coords = np.array(cam_coords).reshape((4,1))
    print(u, v)
    print(cam_coords)
    print(transformation @ cam_coords)

    u, v = (200, 300)
    cam_coords = deproject_pixel_to_point(depth_intrinsics, (u,v), DEPTH_SCALE * depth_img[v, u])
    print("other point img")
    print("intrinsics")
    print(f"fx {depth_intrinsics.fx}, fy {depth_intrinsics.fy}, cx {depth_intrinsics.ppx} cy {depth_intrinsics.ppy}")
    print("u, v")
    print((u, v))
    print("depth")
    print(DEPTH_SCALE * depth_img[v, u])

    cam_coords.append(1)
    cam_coords = np.array(cam_coords).reshape((4,1))
    print(u, v)
    print(cam_coords)
    print(transformation @ cam_coords)
    

if __name__ == "__main__":
    main()