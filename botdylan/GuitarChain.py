import rclpy
import numpy as np
from std_msgs.msg import String
from urdf_parser_py.urdf import Robot
from botdylan.TransformHelpers import *

class GuitarChain:
    def __init__(self, node, baseframe, tipframe):
        """
        Initialize the kinematic chain with fixed transformations from baseframe (world) to fret and string frames.
        """
        self.node = node
        self.steps = []  # To store fixed transformation steps
        self.fret_positions = {}  # To store global fret positions
        self.string_positions = {}  # To store string positions

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

        # Parse the URDF to find fixed transformations between frames (frets and strings)
        self.extract_fret_positions(robot)
        self.extract_string_positions(robot)

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

    def extract_fret_positions(self, robot):
        """
        Extract the positions of the frets from the URDF.
        Assumes that the fret positions are defined by frames/links in the URDF.
        """
        current_transform = np.eye(4)  # Start with identity matrix (world frame)

        # Add the transformation from world to neck
        neck_to_world_transform = None
        for joint in robot.joints:
            if joint.name == 'neck_to_world':
                neck_to_world_transform = self._transform_from_urdf_origin(joint.origin)
                current_transform = current_transform @ neck_to_world_transform  # Apply to get the neck position
                self.info(f"World to neck transform: {current_transform}")

        # Check if the neck_to_world transformation is found
        if neck_to_world_transform is None:
            self.error("neck_to_world transformation not found in the URDF")

        # Now, accumulate the fret positions based on subsequent transformations
        for joint in robot.joints:
            # Filter joints related to frets (e.g., 'fret0_to_fret1', 'fret1_to_fret2', etc.)
            if "fret" in joint.name.lower():
                # Get the transformation matrix for this joint's origin
                fret_transform = self._transform_from_urdf_origin(joint.origin)

                # Accumulate the global position of each fret by applying the current transform
                current_transform = current_transform @ fret_transform  # Multiply to get global position

                # Store the global position of this fret (only translation part)
                fret_name = joint.name.split("_to_")[0]  # Extract fret name (e.g., 'fret0')
                fret_position = current_transform[:3, 3]  # Extract position (last column)
                self.fret_positions[fret_name] = fret_position
                self.info(f"Fret {fret_name} global position: {fret_position}")

    def extract_string_positions(self, robot):
        """
        Extract the positions of the strings from the URDF.
        Assumes that the string positions are defined by frames/links in the URDF.
        """
        current_transform = np.eye(4)  # Start with identity matrix (world frame)

        # Add the transformation from world to neck
        for joint in robot.joints:
            if joint.name == 'neck_to_world':
                current_transform = current_transform @ self._transform_from_urdf_origin(joint.origin)
                self.info(f"World to neck transform: {current_transform}")

        # Now, accumulate the string positions based on subsequent transformations
        for joint in robot.joints:
            # For each string (e.g., 'high_e_to_neck', 'high_e_to_b', etc.), apply the transformation
            if joint.name == 'high_e_to_neck':
                current_transform = current_transform @ self._transform_from_urdf_origin(joint.origin)
                string_position = current_transform[:3, 3]
                self.string_positions['str_high_e'] = string_position
                self.info(f"High E string position: {string_position}")

            elif joint.name == 'high_e_to_b':
                current_transform = current_transform @ self._transform_from_urdf_origin(joint.origin)
                string_position = current_transform[:3, 3]
                self.string_positions['str_b'] = string_position
                self.info(f"B string position: {string_position}")

            elif joint.name == 'b_to_g':
                current_transform = current_transform @ self._transform_from_urdf_origin(joint.origin)
                string_position = current_transform[:3, 3]
                self.string_positions['str_g'] = string_position
                self.info(f"G string position: {string_position}")

            elif joint.name == 'g_to_d':
                current_transform = current_transform @ self._transform_from_urdf_origin(joint.origin)
                string_position = current_transform[:3, 3]
                self.string_positions['str_d'] = string_position
                self.info(f"D string position: {string_position}")

            elif joint.name == 'd_to_a':
                current_transform = current_transform @ self._transform_from_urdf_origin(joint.origin)
                string_position = current_transform[:3, 3]
                self.string_positions['str_a'] = string_position
                self.info(f"A string position: {string_position}")

            elif joint.name == 'a_to_low_e':
                current_transform = current_transform @ self._transform_from_urdf_origin(joint.origin)
                string_position = current_transform[:3, 3]
                self.string_positions['str_low_e'] = string_position
                self.info(f"Low E string position: {string_position}")
    
    def get_fret_positions(self):
        """
        Get the global positions of all the frets from fret_0 to fret_n (based on the URDF).
        Returns a dictionary with fret names as keys and positions as values.
        """
        return self.fret_positions
    
    def get_string_positions(self):
        """
        Get the global positions of all the strings.
        Returns a dictionary with string names as keys and positions as values.
        """
        return self.string_positions
