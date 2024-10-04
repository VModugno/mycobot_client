import os

import cv2
import open3d as o3d
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





def compute_transform_matrix(CUBE_WORLD_POINTS,CUBE_CAMERA_COORDS ):
    source_point_cloud = o3d.geometry.PointCloud()
    source_point_cloud.points = o3d.utility.Vector3dVector(CUBE_WORLD_POINTS) 
    target_point_cloud = o3d.geometry.PointCloud()
    target_point_cloud.points = o3d.utility.Vector3dVector(CUBE_CAMERA_COORDS)

    # for the correspondences the two point clouds are already aligned
    corr = np.zeros((len(CUBE_WORLD_POINTS), 2))
    for i in range(len(CUBE_WORLD_POINTS)):
        corr[:, 0] = i
        corr[:, 1] = i
    


    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    trans_init = p2p.compute_transformation(source_point_cloud, target_point_cloud,
                                            o3d.utility.Vector2iVector(corr))

    # point-to-point ICP for refinement
    print("Perform point-to-point ICP refinement")
    max_iteration = 1000
    criteria = o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6, relative_rmse=1e-6, max_iteration=max_iteration)
    threshold = 0.03  # 3cm distance threshold
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_point_cloud, target_point_cloud, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),criteria=criteria)

    return reg_p2p.transformation


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

    full_extrinsics = None

    with open(os.path.join(file_dir, intrinsics_name), 'rb') as f:
        full_extrinsics = np.load(f)
    
    with open(os.path.join(file_dir, color_img_name_npy), 'rb') as f:
        color_img = np.load(f)
    
    with open(os.path.join(file_dir, depth_img_npy), 'rb') as f:
        depth_img = np.load(f)

    # color_img = cv2.imread(os.path.join(file_dir, color_img_name))
    # depth_img = cv2.imread(os.path.join(file_dir, depth_img_name))
    fx = full_extrinsics[0, 0]
    fy = full_extrinsics[1, 1]
    cx = full_extrinsics[0, 2]
    cy = full_extrinsics[2, 2]
    depth_intrinsics = Intrinsics(fx, fy, cx, cy)

    fig = plt.figure()
    ax = plt.subplot()
    ax.imshow(color_img, cmap="gray")
    ax.set_ylabel("v")
    ax.set_xlabel("u")
    # get checkerboard
    ret, corners = cv2.findChessboardCorners(color_img, (N_CORNERS_X, N_CORNERS_Y))
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

    points_global = np.hstack((Xg[:, None], Yg[:, None], np.zeros((Xg.shape[0], 1))))

    print("global points")
    print(np.array_str(points_global))

    points_camera = np.zeros((corners.shape[0], 3))

    for i in range(len(corners)):
        u, v = corners[i]
        depth = depth_img[int(v), int(u)] * DEPTH_SCALE
        res = deproject_pixel_to_point(depth_intrinsics, [u, v], depth)
        points_camera[i, :] = res



    
    transformation=compute_transform_matrix(points_global, points_camera)

    print(transformation)

    p1_xyz_camera = points_camera[0]
    p1_xyz_global = points_global[0]
    p1_xyz_recreated_camera = transformation @ np.array([p1_xyz_global[0], p1_xyz_global[1], p1_xyz_global[2], 1])

    print("xyz global")
    print(p1_xyz_global)
    print("xyz cam")
    print(p1_xyz_camera)
    print("camera recreated")
    print(p1_xyz_recreated_camera)

    global_recreated = transformation.T @  np.array([p1_xyz_camera[0], p1_xyz_camera[1], p1_xyz_camera[2], 1])
    print("global recreated from cam coords")
    print(global_recreated)

    print("inverting transform")

    p1_xyz_recreated_camera = transformation.T @ np.array([p1_xyz_global[0], p1_xyz_global[1], p1_xyz_global[2], 1])

    print("xyz global")
    print(p1_xyz_global)
    print("xyz cam")
    print(p1_xyz_camera)
    print("camera recreated")
    print(p1_xyz_recreated_camera)

    global_recreated = transformation @  np.array([p1_xyz_camera[0], p1_xyz_camera[1], p1_xyz_camera[2], 1])
    print("global recreated from cam coords")
    print(global_recreated)
    

if __name__ == "__main__":
    main()