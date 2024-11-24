#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        # Set up the intermediate kinematic chain objects.
        self.chain5 = KinematicChain(
            node, 'world', 'link5', self.jointnames()[0:5])
        self.chain4 = KinematicChain(
            node, 'world', 'link4', self.jointnames()[0:4])