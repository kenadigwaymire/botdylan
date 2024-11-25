import rclpy
from math import pi
from GeneratorNode import GeneratorNode
from KinematicChain import KinematicChain

#
#   Trajectory Class
#
class Trajectory:
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain objects.
        # RIGHT HAND
        self.rh_pointer = KinematicChain(node, 'world', 'tip', self.jointnames()[0:6])
        self.rh_middle = KinematicChain(node, 'world', 'tip', self.jointnames()[0:2] + self.jointnames()[6:10])
        self.rh_ring = KinematicChain(node, 'world', 'tip', self.jointnames()[0:2] + self.jointnames()[10:14])
        self.rh_pinky = KinematicChain(node, 'world', 'tip', self.jointnames()[0:2] + self.jointnames()[14:19])
        self.rh_thumb = KinematicChain(node, 'world', 'tip', self.jointnames()[0:2] + self.jointnames()[19:24])
        
        # LEFT HAND
        self.lh_pointer = KinematicChain(node, 'world', 'tip', self.jointnames()[24:30])
        self.lh_middle = KinematicChain(node, 'world', 'tip', self.jointnames()[24:26] + self.jointnames()[30:34])
        self.lh_ring = KinematicChain(node, 'world', 'tip', self.jointnames()[24:26] + self.jointnames()[34:38])
        self.lh_pinky = KinematicChain(node, 'world', 'tip', self.jointnames()[24:26] + self.jointnames()[38:43])
        self.lh_thumb = KinematicChain(node, 'world', 'tip', self.jointnames()[24:26] + self.jointnames()[43:48])
        
        # Initialize joint positions
        self.joint_positions = {name: 0.0 for name in self.jointnames()}
    
    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return [
            # -------------------- Right Hand --------------------
            # wrist
            "rh_WRJ2", "rh_WRJ1",
            "rh_FFJ4", "rh_FFJ3", "rh_FFJ2", "rh_FFJ1",
            "rh_MFJ4", "rh_MFJ3", "rh_MFJ2", "rh_MFJ1",
            "rh_RFJ4", "rh_RFJ3", "rh_RFJ2", "rh_RFJ1",
            "rh_LFJ5", "rh_LFJ4", "rh_LFJ3", "rh_LFJ2", "rh_LFJ1",
            "rh_THJ5", "rh_THJ4", "rh_THJ3", "rh_THJ2", "rh_THJ1",
            # -------------------- Left Hand ---------------------
            # wrist
            "lh_WRJ2", "lh_WRJ1",
            "lh_FFJ4", "lh_FFJ3", "lh_FFJ2", "lh_FFJ1",
            "lh_MFJ4", "lh_MFJ3", "lh_MFJ2", "lh_MFJ1",
            "lh_RFJ4", "lh_RFJ3", "lh_RFJ2", "lh_RFJ1",
            "lh_LFJ5", "lh_LFJ4", "lh_LFJ3", "lh_LFJ2", "lh_LFJ1",
            "lh_THJ5", "lh_THJ4", "lh_THJ3", "lh_THJ2", "lh_THJ1"
        ]
    
    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        # No movement: return the initial joint positions as-is
        qd = [self.joint_positions[joint] for joint in self.jointnames()]
        return qd

#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Initialize the generator node for 100Hz updates, using the above
    # Trajectory class.
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
