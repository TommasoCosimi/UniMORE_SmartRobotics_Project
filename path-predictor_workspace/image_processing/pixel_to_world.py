import os
import json
import logging
from typing import Tuple
from math import sin as s, cos as c, tan
import numpy as np

def compute_intrinsic_matrix(
    camera_intrinsics: list,
    logger: logging.Logger
) -> np.ndarray:
    """
    Computes the intrinsic camera parameters matrix.
    It maps pixel values to points in the space.

    Parameters
    ----------
    camera_intrinsics : list
        List of intrinsic characteristics of the camera.
        They should be provided as a list.
        `[
            [width, height], # Resolution in pixels
            [pixel_physical_width, pixel_physical_height], # Pixel aspect ratio
            Horizontal FOV
        ]`

    logger : logging.Logger
        Used to print info on the console.

    Raises
    ------
    ValueError
        For values below or equal zero in any of the required parameters.

    Returns
    -------
    camera_intrinsic_parameters_matrix : np.ndarray
        Intrinsic camera parameters matrix.
    """

    logger.info("Computing the intrinsic camera parameters matrix")
    # Unpack the data
    pixel_resolution = camera_intrinsics[0]
    if (pixel_resolution[0] <= 0) or (pixel_resolution[1] <= 0):
        raise ValueError("Invalid sensor resolution values.")
    pixel_phisical_width, pixel_phisical_height = camera_intrinsics[1]
    if (pixel_phisical_width <= 0) or (pixel_phisical_height <= 0):
        raise ValueError("Invalid pixels physical dimension values.")
    horizontal_fov = camera_intrinsics[2]
    if (horizontal_fov <= 0):
        raise ValueError("Invalid horizontal FOV value.")
    # Compute the intrinsic parameters matrix
    principal_point = [
        pixel_resolution[0]/2,
        pixel_resolution[1]/2
    ]
    focal_length_x = (pixel_resolution[0] / 2.0) / tan(horizontal_fov / 2.0)
    focal_length_y = focal_length_x * (pixel_phisical_width / pixel_phisical_height)
    camera_intrinsic_parameters_matrix = np.array(
        [
            [focal_length_x, 0, principal_point[0]],
            [0, focal_length_y, principal_point[1]],
            [0, 0, 1]
        ]
    )
    logger.info(f"Intrinsic camera parameters matrix: \n{camera_intrinsic_parameters_matrix}")
    return camera_intrinsic_parameters_matrix


def compute_extrinsic_matrix(
    camera_extrinsics: list,
    logger: logging.Logger
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Computes the intrinsic camera parameters matrix.
    Used to change reference frame (from the camera one to world one).

    Parameters
    ----------
    camera_extrinsics : list
        List of extrinsic characteristics of the camera.
        To be provided as a list of lists.
            `[
                camera_position,
                camera_rotation
            ]`
            Where:
                * camera_position: list
                    Coordinates of the camera viewfinder in the real world with respect to the origin.
                    They should be provided as a triplet of coordinates: `[x, y, z]`
                * camera_rotation: list
                    Rotation of the camera viewfinder in the real world with respect to the origin.
                    They should be provided as a triplet of RPY angles: `[roll, pitch, yaw]`
        
        And thus the complete argument would be:
        `[
            [x, y, z],
            [roll, pitch, yaw]
        ]`

    logger : logging.Logger
        Used to print info on the console.

    Returns
    -------
    camera_rotation_matrix : np.ndarray
        Rotation matrix of the camera reference frame with respect to the world reference frame.

    camera_translation_vector : np.ndarray
        Translation of the camera reference frame with respect to the world reference frame.
    """

    logger.info("Computing the extrinsic camera parameters matrix")
    # Unpack the data
    ox, oy, oz = camera_extrinsics[0]
    r, p, y = camera_extrinsics[1]
    # Compute the rotation matrix and translation vector
    camera_rotation_matrix = np.array(
        [
            [c(y)*c(p), (c(y)*s(p)*s(r))-(s(y)*c(r)), (c(y)*s(p)*c(r))-(s(y)*s(r))],
            [s(y)*c(p), (s(y)*s(p)*s(r))-(c(y)*c(r)), (s(y)*s(p)*c(r))-(c(y)*s(r))],
            [-s(p), c(p)*s(r), c(p)*c(r)]
        ]
    )
    camera_translation_vector = np.array(
        [ox, oy, oz]
    )
    logger.info(f"Camera rotation matrix: \n{camera_rotation_matrix}")
    logger.info(f"Camera translation vector: \n{camera_translation_vector}")
    return camera_rotation_matrix, camera_translation_vector


def pixel_to_coordinates(
    images_dir: str,
    trajectory_computation_dir: str,
    depth_data_filename: str,
    obj_contour_pixels_filename: str,
    obj_contour_coordinates_filename: str,
    camera_intrinsics: list,
    camera_extrinsics: list
) -> None:
    """ Converts pixels to real world positions.
    Intrinsic and extrinsic parameters of the camera will be used to calibrate it.

    Parameters
    ----------
    images_dir : str
        Directory where the images are stored.

    trajectory_computation_dir : str
        Directory where the objects position data is stored.

    depth_data : str
        Name of the depth data file.

    obj_contour_pixels_filename : str
        Name of the objects contours file containing pixel coordinates.

    obj_contour_coordinates_filename : str
        Name of the objects contours file to which results will be dumped.
        
    camera_intrinsics : list
        Intrinsic characteristics of the camera.
        They should be provided as a list.
        `[
            [width, height], # Resolution
            [pixel_width, pixel_height], # Pixel aspect ratio
            Horizontal FOV
        ]`

    camera_extrinsics : list
        Extrinsic characteristics of the camera.
        To be provided as a list of lists.
            `[
                camera_position,
                camera_rotation
            ]`
            Where:
                * camera_position: list
                    Coordinates of the camera viewfinder in the real world with respect to the origin.
                    They should be provided as a triplet of coordinates: `[x, y, z]`
                * camera_rotation: list
                    Rotation of the camera viewfinder in the real world with respect to the origin.
                    They should be provided as a triplet of RPY angles: `[roll, pitch, yaw]`
        
        And thus the complete argument would be:
        `[
            [x, y, z],
            [roll, pitch, yaw]
        ]`

    Raises
    ------
    FileExistsError
        If needed files are not found.

    Returns
    -------
    None
        Results are dumped into `/trajectory_computation_dir/obj_contour_coordinates_filename`.
    """
    # Logger initialization
    logger = logging.getLogger(__name__)
    formatter = logging.Formatter(
        fmt="{asctime} - image_processing/pixel_to_world.py - {levelname} - {message}",
        style="{",
        datefmt="%Y-%m-%d %H:%M"
    )
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.setLevel("DEBUG")
    logger.addHandler(console_handler)
    # Gather the needed data
    logger.info(f"Gathering the needed data")
    ## Depth data
    try:
        if not os.path.exists(os.path.join(images_dir, depth_data_filename)):
            raise FileExistsError(f"The file {images_dir}/{depth_data_filename} does not exist")
    except FileExistsError as fee:
        logger.error(str(fee))
        raise
    depth_data = np.load(os.path.join(images_dir, depth_data_filename))
    ## Object Contours
    try:
        if not os.path.exists(os.path.join(trajectory_computation_dir, obj_contour_pixels_filename)):
            raise FileExistsError(f"The file {trajectory_computation_dir}/{obj_contour_pixels_filename} does not exist")
    except FileExistsError as fee:
        logger.error(str(fee))
        raise
    with open(os.path.join(trajectory_computation_dir, obj_contour_pixels_filename), "r") as objects_contours_file:
        object_contours = json.load(objects_contours_file)
    # Compute the needed transformations
    ## Compute the Intrinsic Parameters Matrix for the camera
    k = compute_intrinsic_matrix(
        camera_intrinsics,
        logger
    )
    ## Compute the Extrinsic Parameters Matrix for the camera
    rotation_matrix, translation_vector = compute_extrinsic_matrix(
        camera_extrinsics,
        logger
    )
    # Compute correspondance between object contour's pixels and real world coordinates
    logger.info(f"Computing the correspondance between pixels and real world points")
    object_contours_coordinates = []
    k_inv = np.linalg.inv(k)
    for point in object_contours:
        u, v = point
        w = depth_data[v, u]
        coordinates_wrt_camera = w * (k_inv @ np.array([u, v, 1.0]))
        coordinates_wrt_worldframe = rotation_matrix @ coordinates_wrt_camera + translation_vector
        ## Round the results to have millimetric precision
        np.round(
            a=coordinates_wrt_worldframe,
            decimals=3,
            out=coordinates_wrt_worldframe
        )
        object_contours_coordinates.append(coordinates_wrt_worldframe.tolist())
    # Dump the results into a json file
    logger.info(f"Saving the results")
    if not os.path.exists(trajectory_computation_dir):
        os.makedirs(trajectory_computation_dir)
    with open(os.path.join(trajectory_computation_dir, obj_contour_coordinates_filename), "w") as output_file:
        json.dump(
            obj=object_contours_coordinates,
            fp=output_file,
            indent=4
        )