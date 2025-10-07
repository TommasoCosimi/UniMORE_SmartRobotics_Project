from math import pi
from image_processing import contour_detector, duplicate_remover, pixel_to_world

# Parameters
camera_intrinsics = [
    [1024, 768], # Camera resolution in pixels
    [0.00000224, 0.00000224], # Pixel physical width and height (in meters)
    0.30 # Horizontal FOV in radiants
] # Parameters gathered from the depth camera's XACRO file
camera_extrinsics = [
    [0.50, 0.00, 2.75], # Viewfinder position wrt the World Frame
    [-pi, 0, pi/2.0] # Viewfinder rotation wrt the World Frame
]

if __name__ == "__main__":
    contour_detector.detect_contours(
        images_dir="/common/image_processing",
        trajectory_computation_dir="/common/trajectory_planning",
        rgb_image_filename="camera_image.png",
        obj_contour_pixels_filename="relevant_contours_pixels.json"
    )
    pixel_to_world.pixel_to_coordinates(
        images_dir="/common/image_processing",
        trajectory_computation_dir="/common/trajectory_planning",
        depth_data_filename="depth_data.npy",
        obj_contour_pixels_filename="relevant_contours_pixels.json",
        obj_contour_coordinates_filename="relevant_contours_coordinates.json",
        camera_intrinsics=camera_intrinsics,
        camera_extrinsics=camera_extrinsics
    )
    duplicate_remover.remove_duplicates(
        trajectory_computation_dir="/common/trajectory_planning",
        obj_contour_coordinates_filename="relevant_contours_coordinates.json",
        obj_contour_coordinates_cleared_filename="relevant_contours_cleared_coordinates.json"
    )