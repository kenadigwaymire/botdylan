import rclpy # type: ignore
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# import sys
# print(sys.path) # To debug pathing

# Grab the utilities
from botdylan.GeneratorNode      import GeneratorNode
from botdylan.TransformHelpers   import *
from botdylan.TrajectoryUtils    import *

# Grab the general fkin
from botdylan.KinematicChain     import KinematicChain

NUM_STRINGS = 6
STRING_NOTES = {0 : 'e_high', 1 : 'b', 2 : 'g', 3 : 'd', 4 : 'a', 5 : 'e_low'}

# TODO: Flesh out a function to read the song that gets initialized when the tajectory
# gets initialized.
def song_info(song):
    # Eventually change these to be pulled or calculated from a song file
    T = 3
    chords = []
    strumming_pattern = []
    return [T, chords, strumming_pattern]

# TODO: draw fretboard for guitar and map positions to chords

class Fretboard():
    def __init__(self, num_frets, dx, dy):
        self.dx = dx
        self.dy = dy
        self.width = NUM_STRINGS * dy
        self.length = num_frets * dx
        self.fretboard = [[(i, j) for j in range(num_frets)] for i in range(NUM_STRINGS)]

    def get_pos_desired(self, string_des, fret_des):
        return (string_des * self.dy, (fret_des * self.dx) + self.dx / 2)
    
    def get_coord_from_pos(self, curr_pos):
        return (curr_pos[0] / self.dy, (curr_pos[1] - (self.dx / 2)) / self.dx)
    
    def get_val_range_of_fret(self):
        return None # temporary for debugging
        
#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain objects.
        # RIGHT HAND
        self.rh_pointer = KinematicChain(node, 'world', 'rh_fftip',
                            self.jointnames()[0:6])
        self.rh_middle = KinematicChain(node, 'world', 'rh_mftip',
                            self.jointnames()[0:2]+self.jointnames()[6:10])
        self.rh_ring = KinematicChain(node, 'world', 'rh_rftip', 
                            self.jointnames()[0:2]+self.jointnames()[10:14])
        self.rh_pinky = KinematicChain(node, 'world', 'rh_lftip',
                            self.jointnames()[0:2]+self.jointnames()[14:19])
        self.rh_thumb = KinematicChain(node, 'world', 'rh_thtip',
                            self.jointnames()[0:2]+self.jointnames()[19:24])
        
        # LEFT HAND
        self.lh_pointer = KinematicChain(node, 'world', 'lh_fftip',
                            self.jointnames()[24:30])
        self.lh_middle = KinematicChain(node, 'world', 'lh_mftip',
                            self.jointnames()[24:26]+self.jointnames()[30:34])
        self.lh_ring = KinematicChain(node, 'world', 'lh_rftip', 
                            self.jointnames()[24:26]+self.jointnames()[34:38])
        self.lh_pinky = KinematicChain(node, 'world', 'lh_lftip',
                            self.jointnames()[24:26]+self.jointnames()[38:43])
        self.lh_thumb = KinematicChain(node, 'world', 'lh_thtip',
                            self.jointnames()[24:26]+self.jointnames()[43:48])
        
        # Init joint values (doesnt work rn cause chain is <6 and len(jointnames) = 48)
        self.qd = np.zeros(len(self.jointnames())) # which is declared first?

        # Other params
        self.lam = 20
        # self.pdlast = np.copy(self.p0)
        # self.Rdlast = np.copy(self.R0)
        
        
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
        # (pd, vd) = goto(t, T, nextChord, prevChord)
        # xddot = vd

        [ptips, Rtips, Jv, Jw] = np.vstack((
            self.rh_pointer.fkin(self.qd[0:6]), 
            self.rh_middle.fkin(np.concatenate((self.qd[0:2],self.qd[6:10]))), 
            self.rh_ring.fkin(np.concatenate((self.qd[0:2],self.qd[10:14]))), 
            self.rh_pinky.fkin(np.concatenate((self.qd[0:2],self.qd[14:19]))), 
            self.rh_thumb.fkin(np.concatenate((self.qd[0:2],self.qd[19:24]))), 

            self.lh_pointer.fkin(self.qd[24:30]), 
            self.lh_middle.fkin(np.concatenate((self.qd[24:26],self.qd[30:34]))), 
            self.lh_ring.fkin(np.concatenate((self.qd[24:26],self.qd[34:38]))), 
            self.lh_pinky.fkin(np.concatenate((self.qd[24:26],self.qd[38:43]))), 
            self.lh_thumb.fkin(np.concatenate((self.qd[24:26],self.qd[43:48]))), 
            ))
        J = np.vstack((Jv, Jw))
        
        # errp = ep(self.pdlast, ptips)
        # # errR = eR(self.Rdlast, Rtips)
        # # err = np.concatenate((errp, errR))
        
        # qdlast = self.qd
        # qddot = Jpinv @ (xddot + self.lam * errp)

        # self.qd += qddot * dt
        # qd = self.qd
        # pdlast = pd
        # Rdlast = Rd

        z_vec = np.zeros(len(self.jointnames()))

        # return (qd, qddot, pd, vd, Rd, wd)
        return [z_vec, z_vec, np.zeros(3), np.zeros(3), Reye(), np.zeros(3)]

#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Choose the song you want to play:
    song = "song_name"

    # Initialize the generator node for 100Hz udpates, using the above
    # Trajectory class.
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
