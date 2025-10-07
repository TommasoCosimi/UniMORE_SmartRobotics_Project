import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2
from cv_bridge import CvBridge

# Parameters
img_processing_dir = "/common/image_processing"


class DepthCameraSubscriber(Node):
    """
    Node which subscribes the three topics of the depth_camera element:
    * /depth_camera/image_raw
    * /depth_camera/depth/image_raw
    * /depth_camera/points

    and uses them to gather both image and depth data.
    """

    def __init__(self):
        super().__init__("depth_camera_subscriber")
        self.bridge = CvBridge()
        # Control variables
        self.image_received = False
        self.image_depth_received = False
        self.depth_received = False
        # Subscribers to the image and PointCloud topics
        self.image_subscriber = self.create_subscription(
            Image,
            "/depth_camera/image_raw",
            self.image_callback,
            10
        )
        self.depthimage_subscriber = self.create_subscription(
            Image,
            "/depth_camera/depth/image_raw",
            self.depthimage_callback,
            10
        )
        self.depth_subscriber = self.create_subscription(
            PointCloud2,
            "/depth_camera/points",
            self.depth_callback,
            10
        )


    def image_callback(
        self,
        msg
    ) -> None:
        """
        Gathers the image from the desired topic and saves it.

        Parameters
        ----------
        msg : Unknown
            Message coming from the ROS2 Topic.

        Returns
        -------
        None
            The results are passed to the support function to save image files.
        """
        if self.image_received == True:
            return
        # Gather the data
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.image_received = True
        # Save the results
        self.get_logger().info("Image received")
        self.save_img(
            image=image,
            filename="camera_image.png"
        )
    

    def depthimage_callback(
        self,
        msg
    ) -> None:
        """
        Gathers the depth image from the desired topic and saves it, along with a normalized version.

        Parameters
        ----------
        msg : Unknown
            Message coming from the ROS2 Topic.

        Returns
        -------
        None
            The results are passed to the support function to save image files.
        """
        if self.image_depth_received == True:
            return
        # Gather the data
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        self.image_depth_received = True
        self.get_logger().info("Depth Image received")
        # Compute the normalized version
        depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        # Save the results
        self.save_img(
            image=depth_image,
            filename="depth_camera_image.png"
        )
        self.save_img(
            image=depth_image_normalized,
            filename="depth_camera_image_normalized.png"
        )


    def depth_callback(
        self,
        msg
    ) -> None:
        """
        Gathers the depth cloud from the desired topic and converts it to a numpy array.

        Parameters
        ----------
        msg : Unknown
            Message coming from the ROS2 Topic.

        Returns
        -------
        None
            The results are saved into `/img_processing_dir/depth_data.npy`.
        """
        if self.depth_received == True:
            return
        # Gather the data
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        self.depth_received = True
        self.get_logger().info("PointCloud2 received")
        # Filter just the depth data of each point
        depth_array = np.array([point[2] for point in points])
        # Reshape the array to match the image
        depth_data = depth_array.reshape((msg.height, msg.width))
        # Save the results
        if not os.path.exists(img_processing_dir):
            os.makedirs(img_processing_dir)
        np.save(f"{img_processing_dir}/depth_data.npy", depth_data)
        self.get_logger().info("Depth Data saved as depth_data.npy")


    def save_img(
        self,
        image: np.ndarray,
        filename: str
    ) -> None:
        """
        Support function to save images.

        Parameters
        ----------
        image : np.ndarray
            OpenCV Image which has to be saved.

        filename : str
            Filename of the saved image.

        Returns
        -------
        None
            The function saves the desired image in `/img_processing_dir/filename`.
        """

        if not os.path.exists(img_processing_dir):
            os.makedirs(img_processing_dir)
        cv2.imwrite(os.path.join(img_processing_dir, filename), image)
        self.get_logger().info(f"Image saved as {filename}")
        
    def run(self):
        # Runs the node and shuts it down once it finishes
        while rclpy.ok() and not (self.image_received and self.image_depth_received and self.depth_received):
            rclpy.spin_once(self)
        self.get_logger().info("All the necessary data has been saved. Shutting down...")
        self.destroy_node()
        rclpy.shutdown

def main(args=None):
    rclpy.init(args=args)
    depth_camera_subscriber = DepthCameraSubscriber()
    depth_camera_subscriber.run()
    
if __name__ == '__main__':
    main()
