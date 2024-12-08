import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from GeneratorNode      import GeneratorNode
from TransformHelpers   import *
from TrajectoryUtils    import *

# Grab the general fkin
from KinematicChain     import KinematicChain

NUM_STRINGS = 6
STRING_NOTES = {0 : 'e_high', 1 : 'b', 2 : 'g', 3 : 'd', 4 : 'a', 5 : 'e_low'}

# TODO: Flesh out a function to read the song that gets initialized when the trajectory
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


#
#   Trajectory Class
#
class Trajectory:
    # Initialization.
    def __init__(self, song, node):
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
        
        # Other params
        self.lam = 20
        self.song = song

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return [
            # -------------------- Right Hand --------------------
            "rh_WRJ2", "rh_WRJ1",
            "rh_FFJ4", "rh_FFJ3", "rh_FFJ2", "rh_FFJ1",
            "rh_MFJ4", "rh_MFJ3", "rh_MFJ2", "rh_MFJ1",
            "rh_RFJ4", "rh_RFJ3", "rh_RFJ2", "rh_RFJ1",
            "rh_LFJ5", "rh_LFJ4", "rh_LFJ3", "rh_LFJ2", "rh_LFJ1",
            "rh_THJ5", "rh_THJ4", "rh_THJ3", "rh_THJ2", "rh_THJ1",
            # -------------------- Left Hand ---------------------
            "lh_WRJ2", "lh_WRJ1",
            "lh_FFJ4", "lh_FFJ3", "lh_FFJ2", "lh_FFJ1",
            "lh_MFJ4", "lh_MFJ3", "lh_MFJ2", "lh_MFJ1",
            "lh_RFJ4", "lh_RFJ3", "lh_RFJ2", "lh_RFJ1",
            "lh_LFJ5", "lh_LFJ4", "lh_LFJ3", "lh_LFJ2", "lh_LFJ1",
            "lh_THJ5", "lh_THJ4", "lh_THJ3", "lh_THJ2", "lh_THJ1"
        ]
    
    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        [ptips, Rtips, Jv, Jw] = np.vstack((
            self.rh_pointer.fkin(self.qd), 
            self.rh_middle.fkin(self.qd), 
            self.rh_ring.fkin(self.qd), 
            self.rh_pinky.fkin(self.qd), 
            self.rh_thumb.fkin(self.qd), 

            self.lh_pointer.fkin(self.qd), 
            self.lh_middle.fkin(self.qd), 
            self.lh_ring.fkin(self.qd), 
            self.lh_pinky.fkin(self.qd), 
            self.lh_thumb.fkin(self.qd), 
        ))
        
        J = np.vstack((Jv, Jw))

        z_vec = np.zeros(len(self.jointnames()))
        return [z_vec, z_vec, np.zeros(3), np.zeros(3), Reye(), np.zeros(3)]


#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Choose the song you want to play:
    song = "song_name"

    # Initialize the generator node for 100Hz updates, using the above
    # Trajectory class.
    generator = GeneratorNode('generator', 100, Trajectory(song, None))  # Pass in song and node

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

# Scrapped Code:
# def get_Jw(self):
#         """
#         Returns the rotational Jacobian of size 27x40 for the robot (both hands).
#         """
#         Jw = np.zeros((27, 40))
#         Jw[0:3, 0:6] = self.rh_ff.fkin(self.qd[0:6])[3]
#         rh_mf_Jw = self.rh_mf.fkin(np.concatenate((self.qd[0:2],self.qd[6:10])))[3]
#         Jw[3:6, 0:2], Jw[3:6, 6:10] = rh_mf_Jw[:,0:2], rh_mf_Jw[:,2:6]
#         rh_rf_Jw = self.rh_rf.fkin(np.concatenate((self.qd[0:2],self.qd[10:14])))[3]
#         Jw[6:9, 0:2], Jw[6:9, 10:14] = rh_rf_Jw[:,0:2], rh_rf_Jw[:,2:6]
#         rh_lf_Jw = self.rh_lf.fkin(np.concatenate((self.qd[0:2],self.qd[14:19])))[3]
#         Jw[9:12, 0:2], Jw[9:12, 14:19] = rh_lf_Jw[:,0:2], rh_lf_Jw[:,2:7]
#         # We don't need the right-hand thumb — we set these to fixed joints
#         # [rh_th_ptip, rh_th_Rtip, rh_th_Jw, rh_th_Jw] = self.rh_thumb.fkin(np.concatenate((self.qd[0:2],self.qd[19:24])))

#         Jw[12:15, 19:26] = self.lh_ff.fkin(self.qd[19:26])[3]
#         lh_mf_Jw = self.lh_mf.fkin(np.concatenate((self.qd[19:22],self.qd[26:30])))[3]
#         Jw[15:18, 19:22], Jw[15:18, 26:30], = lh_mf_Jw[:,0:3], lh_mf_Jw[:,3:7]
#         lh_rf_Jw = self.lh_rf.fkin(np.concatenate((self.qd[19:22],self.qd[30:34])))[3]
#         Jw[18:21, 19:22], Jw[18:21, 30:34] = lh_rf_Jw[:,0:3], lh_rf_Jw[:,3:7]
#         lh_lf_Jw = self.lh_lf.fkin(np.concatenate((self.qd[19:22],self.qd[34:39])))[3]
#         Jw[21:24, 19:22], Jw[21:24, 34:39] = lh_lf_Jw[:,0:3], lh_lf_Jw[:,3:8]
#         lh_th_Jw = self.lh_thumb.fkin(np.concatenate((self.qd[19:22],self.qd[39:40])))[3]
#         Jw[24:27, 19:22], Jw[24:27, 39:40] = lh_th_Jw[:,0:3], lh_th_Jw[:,3:4]
#         return Jw

# def get_Rtips(self):
#         """
#         Returns the orientations of the fingers as an array of size 3x27, representing
#         the 3x3 rotation matrices of each of the 9 fingers used (in the order that
#         they appear in jointnames).
#         """
#         return np.hstack([
#                 self.rh_ff.fkin(self.qd[0:6])[1],
#                 self.rh_mf.fkin(np.concatenate((self.qd[0:2],self.qd[6:10])))[1],
#                 self.rh_rf.fkin(np.concatenate((self.qd[0:2],self.qd[10:14])))[1],
#                 self.rh_lf.fkin(np.concatenate((self.qd[0:2],self.qd[14:19])))[1],

#                 self.lh_ff.fkin(self.qd[19:26])[1],
#                 self.lh_mf.fkin(np.concatenate((self.qd[19:22],self.qd[26:30])))[1],
#                 self.lh_rf.fkin(np.concatenate((self.qd[19:22],self.qd[30:34])))[1],
#                 self.lh_lf.fkin(np.concatenate((self.qd[19:22],self.qd[34:39])))[1],
#                 self.lh_thumb.fkin(np.concatenate((self.qd[19:22],self.qd[39:40])))[1]
#                 ])

# self.R0 = self.get_Rtips()
# self.Rdlast = np.copy(self.R0)
# self.Rdlast = Rd

# # We're not actually interested in the rotations of the fingers, but we 
# abitrarily initialize them here to make the evaluate function compatible
# with the GeneratorNode.py
# Rd = np.zeros((3,27))
# wd = np.zeros(27)]

# # No need to lift the fingers when the song is over.
            # if chord_ct != len(chords):
            #     for i in [2, 5, 8, 11]:
            #         prevChord[i] += 0.01


# # List of decent x and y offsets for each finger  relative to the wrist
#         # (for the left hand) to guess a reasonable position for fingers that
#         # aren't involved in playing the chord.
#         finger_offset = [[-0.033, p0[13]],
#                          [-0.011, p0[16]], 
#                          [0.011, p0[19]], 
#                          [0.033, p0[22] - 0.01]]