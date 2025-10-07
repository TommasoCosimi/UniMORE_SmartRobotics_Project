import os
import json
import logging
import numpy as np
import cv2

def filter_contours(
    contours: tuple[np.ndarray],
    logger: logging.Logger
) -> list:
    """ Contour Filter
    A function which filters needed object contours for the desired task.

    Parameters
    ----------
    contours : tuple
        The contours found in the `detect_contours()` function.
    logger : logging.Logger
        Logger to use to print messages.

    Raises
    ------
    ValueError
        If the chosen contours are not in a valid range.

    Returns
    -------
    filtered_contours : np.ndarray
        Contours of the desired object only
    """
    
    # User prompt for choosing the desired object
    while True:
        try:
            chosen_object = int(
                input(
                    "Select the object to choose by its index: "
                )
            )
            if (chosen_object < 0) or (chosen_object > (len(contours) - 1)):
                logger.warning("Please choose a valid index")
            else:
                break
        except ValueError:
            logger.warning("Please enter the index of the object as a number")
    assert 0 <= chosen_object < len(contours)
    object_contours = contours[chosen_object]
    # Squeeze the contours array dimensions to remove the redundant one
    object_contours = np.squeeze(object_contours)
    return object_contours.tolist()


def detect_contours(
    images_dir: str,
    trajectory_computation_dir: str,
    rgb_image_filename: str,
    obj_contour_pixels_filename: str
) -> None:
    """ Contour Detector
    A function which is able to find objects contours.
    Saves the detection in a file.

    Parameters
    ----------
    images_dir : str
        Directory where the images are stored.

    trajectory_computation_dir : str
        Directory where the objects' position data will be stored.
    
    rgb_image_filename : str
        Name of the RGB Image file coming from the camera.
    
    obj_contour_filename : str
        Name of the object contour file for the result dumping.
    
    camera_calibration : bool
        Makes possible to use this function to calibrate the camera using the passed image.
    
    Raises
    -----
    FileExistsError
        If the desired image file does not exist.

    Returns
    -------
    None
        The function dumps its results in `/trajectory_computation_dir/obj_contour_pixels_filename`.
    """

    # Logger initialization
    logger = logging.getLogger(__name__)
    formatter = logging.Formatter(
        fmt="{asctime} - image_processing/contour_detector.py - {levelname} - {message}",
        style="{",
        datefmt="%Y-%m-%d %H:%M"
    )
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.setLevel("DEBUG")
    logger.addHandler(console_handler)
    # Open the image
    try:
        if not os.path.exists(os.path.join(images_dir, rgb_image_filename)):
            raise FileExistsError(f"The file {images_dir}/{rgb_image_filename} does not exist")
    except FileExistsError as fee:
        logger.error(str(fee))
        raise
    logger.info("Opening RGB image")
    rgb_image:np.ndarray = cv2.imread(
        os.path.join(images_dir, rgb_image_filename)
    ) # pyright: ignore[reportAssignmentType]
    logger.info("Running an Edge Detector on the image")
    # Convert the image to Black/White in order to find edges
    bw_image:np.ndarray = cv2.cvtColor(
        rgb_image,
        cv2.COLOR_BGR2GRAY
    )
    # Threshold the image
    return_value, threshold = cv2.threshold(
        src=bw_image,
        thresh=127,
        maxval=255,
        type=cv2.THRESH_BINARY
    )
    ## Detect contours
    contours, hierarchy = cv2.findContours(
        threshold,
        mode=cv2.RETR_TREE,
        method=cv2.CHAIN_APPROX_NONE
    )
    ## Add a label to each contour
    for i, contour in enumerate(contours):
        x, y , w, h = cv2.boundingRect(contour)
        cv2.putText(
            img=rgb_image,
            text=f"Object_{i}",
            org=(x, y+25),
            fontFace=cv2.FONT_HERSHEY_DUPLEX,
            fontScale=0.75,
            color=(0, 0, 255),
            thickness=1
        )
    # Show the results
    cv2.drawContours(
        image=rgb_image,
        contours=contours,
        contourIdx=-1,
        color=(0, 255, 0),
        thickness=1
    )
    cv2.imshow(f"Detected Objects", rgb_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    # Filter the contours to keep the desired ones only
    contours_of_interest = filter_contours(
        contours=contours,
        logger=logger
    )
    # Dump the results into a json file
    if not os.path.exists(trajectory_computation_dir):
        os.makedirs(trajectory_computation_dir)
    with open(os.path.join(trajectory_computation_dir, obj_contour_pixels_filename), "w") as output_file:
        json.dump(
            obj=contours_of_interest,
            fp=output_file,
            indent=4
        )