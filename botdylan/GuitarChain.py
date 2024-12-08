import rclpy
import numpy as np
from std_msgs.msg import String  # <-- Make sure this is imported
from urdf_parser_py.urdf import Robot
from botdylan.TransformHelpers   import *

class GuitarChain:
    def __init__(self, node, baseframe, tipframe):
        """
        Initialize the kinematic chain with fixed transformations from baseframe to tipframe.
        """
        self.node = node
        self.steps = []  # To store fixed transformation steps

        # Create a temporary subscriber to receive the URDF.
        self.info("Waiting for the URDF to be published...")
        self.urdf = None
        def cb(msg):
            self.urdf = msg.data
        topic = '/robot_description'
        sub = self.node.create_subscription(String, topic, cb, qos_profile=10)
        while self.urdf is None:
            rclpy.spin_once(self.node)
        self.node.destroy_subscription(sub)

        # Convert the URDF string into a Robot object
        robot = Robot.from_xml_string(self.urdf)
        self.info(f"Processing URDF for robot '{robot.name}'")

        # Parse the URDF to find fixed transformations between frames.
        frame = tipframe
        while frame != baseframe:
            joint = next((j for j in robot.joints if j.child == frame), None)
            if joint is None:
                self.error(f"Unable to find joint connecting to '{frame}'")
            frame = joint.parent

            # Store fixed transformations
            self.steps.insert(0, {
                'name': joint.name,
                'Tshift': self._transform_from_urdf_origin(joint.origin)
            })

        self.info(f"URDF has {len(self.steps)} fixed transformations from {baseframe} to {tipframe}")

    def _transform_from_urdf_origin(self, origin):
        """
        Convert the URDF origin into a 4x4 transformation matrix using TransformHelpers.
        """
        translation = origin.xyz  # This is now a list, so just use it directly.
        rotation = origin.rpy  # This is also a list, so we can directly use it.
        return self._build_transform_matrix(translation, rotation)

    def _build_transform_matrix(self, translation, rotation):
        """
        Build a 4x4 transformation matrix from translation and rotation (RPY).
        Using TransformHelpers for rotation handling instead of tf2.
        """
        # Import necessary functions from TransformHelpers.py
        from botdylan.TransformHelpers import R_from_RPY, T_from_Rp

        # Create the rotation matrix from Euler angles (roll, pitch, yaw)
        R = R_from_RPY(rotation[0], rotation[1], rotation[2])

        # Build the transformation matrix using the rotation and translation
        transform_matrix = T_from_Rp(R, np.array(translation))
        return transform_matrix

    def info(self, message):
        """Helper function to log info messages."""
        self.node.get_logger().info(f"KinematicChain: {message}")

    def error(self, message):
        """Helper function to log error messages."""
        self.node.get_logger().error(f"KinematicChain: {message}")
        raise Exception(message)

    def fkin(self):
        """
        Compute the forward kinematics for the kinematic chain with fixed transformations.
        This just applies the fixed transformations sequentially.
        """
        # Start with the identity matrix (base frame).
        T = np.eye(4)

        # Apply all fixed transformations
        for step in self.steps:
            T = T @ step['Tshift']

        # Return the final tip position (translation) and rotation
        tip_position = T[:3, 3]  # The position is the last column
        tip_rotation = T[:3, :3]  # The rotation is the top-left 3x3 submatrix
        return tip_position, tip_rotation
