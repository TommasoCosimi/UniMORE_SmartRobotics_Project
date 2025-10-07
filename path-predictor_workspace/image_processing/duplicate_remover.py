import os
import json
import logging

def remove_duplicates(
    trajectory_computation_dir: str,
    obj_contour_coordinates_filename: str,
    obj_contour_coordinates_cleared_filename: str
) -> None:
    """ Converts pixels to real world positions.
    Intrinsic and extrinsic parameters of the camera will be used to calibrate it.

    Parameters
    ----------
    trajectory_computation_dir : str
        Directory where the objects position data is stored.

    obj_contour_coordinates_filename : str
        Name of the objects contours file with potential duplicates.
    
    obj_contour_coordinates_cleared_filename : str
        Name of the objects contours file with no duplicates to which results will be dumped.

    Raises
    ------
    FileExistsError
        If needed files are not found.

    Returns
    -------
    None
        Results are dumped into `/trajectory_computation_dir/obj_contour_coordinates_cleared_filename`.
    """
    # Logger initialization
    logger = logging.getLogger(__name__)
    formatter = logging.Formatter(
        fmt="{asctime} - image_processing/duplicate_remover.py - {levelname} - {message}",
        style="{",
        datefmt="%Y-%m-%d %H:%M"
    )
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.setLevel("DEBUG")
    logger.addHandler(console_handler)
    # Gather the needed data
    logger.info(f"Gathering the needed data")
    try:
        if not os.path.exists(os.path.join(trajectory_computation_dir, obj_contour_coordinates_filename)):
            raise FileExistsError(f"The file {trajectory_computation_dir}/{obj_contour_coordinates_filename} does not exist")
    except FileExistsError as fee:
        logger.error(str(fee))
        raise
    with open(os.path.join(trajectory_computation_dir, obj_contour_coordinates_filename), "r") as objects_contours_file:
        object_contours = json.load(objects_contours_file)
    # Remove Duplicates
    cleared_points = []
    for idx, point in enumerate(object_contours):
        if idx > 0:
            if point == object_contours[idx-1]:
                logger.info(f"Duplicate point found")
            else:
                cleared_points.append(point)
        else:
            logger.info(f"Adding first point")
            cleared_points.append(point)
    # Dump the results into a json file
    logger.info(f"Saving the results")
    if not os.path.exists(trajectory_computation_dir):
        os.makedirs(trajectory_computation_dir)
    with open(os.path.join(trajectory_computation_dir, obj_contour_coordinates_cleared_filename), "w") as output_file:
        json.dump(
            obj=cleared_points,
            fp=output_file,
            indent=4
        )