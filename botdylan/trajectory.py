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
    chords = [{'G' : [(4, 1), (5, 2), (1, 2), (5, 2)]}]
    strumming_pattern = []
    return [T, chords, strumming_pattern]

# TODO: draw fretboard for guitar and map positions to chords

class Fretboard():
    def __init__(self, num_frets, dx, dy, z):
        self.dx = dx
        self.dy = dy
        self.z = z # string z height
        self.width = NUM_STRINGS * dy
        self.length = num_frets * dx
        self.fretboard = [[(i, j) for j in range(num_frets)] for i in range(NUM_STRINGS)]
    def pd_from_chord(self, chord):
        chord_position = np.copy(chord)
        chord_position = chord_position * np.array([self.dy, self.dx])
        chord_position[:, 1] += self.dx / 2
        chord_position = np.hstack(np.array(list(zip(chord_position[:,0], 
                                                     chord_position[:,1],
                                                     self.z * np.ones(4)))))
        return chord_position
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
        self.q0 =  # write the initial joint positions here
        # self.p0 = np.array([0.0, 0.0, 0.0,
        #                     0.1, 0.0, 0.0,
        #                     0.2, 0.0, 0.0,
        #                     0.3, 0.0, 0.0,
        #                     0.4, 0.0, 0.0,

        #                     12.0, 0.0, 0.0,
        #                     12.1, 0.0, 0.0,
        #                     12.2, 0.0, 0.0,
        #                     12.3, 0.0, 0.0,
        #                     12.4, 0.0, 0.0])
        self.R0 = np.hstack((Reye(), Reye(), Reye(), Reye(), Reye(), 
                             Reye(), Reye(), Reye(), Reye(), Reye()))

        # Other params
        self.lam = 20
        self.pdlast = np.copy(self.p0)
        self.Rdlast = np.copy(self.R0)
        
    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return [
            # -------------------- Right Hand (STRUMMING) --------------------
            "rh_WRJ2", "rh_WRJ1", # wrist
            "rh_FFJ4", "rh_FFJ3", "rh_FFJ2", "rh_FFJ1", # pointer
            "rh_MFJ4", "rh_MFJ3", "rh_MFJ2", "rh_MFJ1", # middle
            "rh_RFJ4", "rh_RFJ3", "rh_RFJ2", "rh_RFJ1", # ring
            "rh_LFJ5", "rh_LFJ4", "rh_LFJ3", "rh_LFJ2", "rh_LFJ1", # pinky
            # We won't be using the right hand thumb — we set these to fixed joints
            # "rh_THJ5", "rh_THJ4", "rh_THJ3", "rh_THJ2", "rh_THJ1",
            # -------------------- Left Hand (FRETTING) ----------------------
            "right_hand_to_left_hand",  # prismatic joint for sliding along the neck
            "lh_WRJ2", "lh_WRJ1", # wrist
            "lh_FFJ4", "lh_FFJ3", "lh_FFJ2", "lh_FFJ1", # pointer
            "lh_MFJ4", "lh_MFJ3", "lh_MFJ2", "lh_MFJ1", # middle
            "lh_RFJ4", "lh_RFJ3", "lh_RFJ2", "lh_RFJ1", # ring
            "lh_LFJ5", "lh_LFJ4", "lh_LFJ3", "lh_LFJ2", "lh_LFJ1", # pinky
            "lh_THJ5", # Keeping one joint to pivot thumb up and down to stay below guitar neck when wrist moves
            # Made these fixed:
            # "lh_THJ4", "lh_THJ3", "lh_THJ2", "lh_THJ1"
            ]
    
    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        # Initialize a guitar with: 21 frets, spaced 1 inch apart, and 
        # 6 strings spaced 0.5 inches apart, at  a height of 0
        fretboard = Fretboard(21, 1, 0.5, 0)

        # Get the beat (T seconds), chords, and strumming pattern
        [T, chords, strumming_pattern] = song_info('some_song')

        nextChord = fretboard.pd_from_chord(chords[0].get('G'))
        nextChord = np.hstack((nextChord, self.p0[12:30]))
        prevChord = self.p0
        (pd, vd) = goto(t, T, prevChord, nextChord)
        Rd = self.R0
        wd = np.zeros(30)
        xddot = np.concatenate((vd, np.zeros(2), wd, np.zeros(2)))

        [rh_ff_ptip, rh_ff_Rtip, rh_ff_Jv, rh_ff_Jw] = self.rh_pointer.fkin(self.qd[0:6])
        [rh_mf_ptip, rh_mf_Rtip, rh_mf_Jv, rh_mf_Jw] = self.rh_middle.fkin(np.concatenate((self.qd[0:2],self.qd[6:10])))
        [rh_rf_ptip, rh_rf_Rtip, rh_rf_Jv, rh_rf_Jw] = self.rh_ring.fkin(np.concatenate((self.qd[0:2],self.qd[10:14])))
        [rh_lf_ptip, rh_lf_Rtip, rh_lf_Jv, rh_lf_Jw] = self.rh_pinky.fkin(np.concatenate((self.qd[0:2],self.qd[14:19])))
        # We don't need the right hand thumb — we set these to fixed joints
        # [rh_th_ptip, rh_th_Rtip, rh_th_Jv, rh_th_Jw] = self.rh_thumb.fkin(np.concatenate((self.qd[0:2],self.qd[19:24])))

        [lh_ff_ptip, lh_ff_Rtip, lh_ff_Jv, lh_ff_Jw] = self.lh_pointer.fkin(self.qd[19:26])
        [lh_mf_ptip, lh_mf_Rtip, lh_mf_Jv, lh_mf_Jw] = self.lh_middle.fkin(np.concatenate((self.qd[19:22],self.qd[26:30])))
        [lh_rf_ptip, lh_rf_Rtip, lh_rf_Jv, lh_rf_Jw] = self.lh_ring.fkin(np.concatenate((self.qd[19:22],self.qd[30:34])))
        [lh_lf_ptip, lh_lf_Rtip, lh_lf_Jv, lh_lf_Jw] = self.lh_pinky.fkin(np.concatenate((self.qd[19:22],self.qd[34:39])))
        [lh_th_ptip, lh_th_Rtip, lh_th_Jv, lh_th_Jw] = self.lh_thumb.fkin(np.concatenate((self.qd[19:22],self.qd[39]))) 

        [ptips, Rtips, errR, Jv, Jw] = [np.hstack((rh_ff_ptip, rh_mf_ptip, 
                                    rh_rf_ptip, rh_lf_ptip, 
                                    lh_ff_ptip, lh_mf_ptip, lh_rf_ptip, 
                                    lh_lf_ptip, lh_th_ptip)),
                                 np.hstack((rh_ff_Rtip, rh_mf_Rtip, 
                                    rh_rf_Rtip, rh_lf_Rtip, 
                                    lh_ff_Rtip, lh_mf_Rtip, lh_rf_Rtip, 
                                    lh_lf_Rtip, lh_th_Rtip)),
                                 np.hstack((eR(self.Rdlast, rh_ff_Rtip),
                                            eR(self.Rdlast, rh_mf_Rtip), 
                                            eR(self.Rdlast, rh_rf_Rtip), 
                                            eR(self.Rdlast, rh_lf_Rtip), 
                                            eR(self.Rdlast), 
                                            eR(self.Rdlast, lh_ff_Rtip), 
                                            eR(self.Rdlast, lh_mf_Rtip), 
                                            eR(self.Rdlast, lh_rf_Rtip), 
                                            eR(self.Rdlast, lh_lf_Rtip), 
                                            eR(self.Rdlast, lh_th_Rtip))),
                                 np.hstack((rh_ff_Jv, rh_mf_Jv, rh_rf_Jv, 
                                    rh_lf_Jv,
                                    lh_ff_Jv, lh_mf_Jv, lh_rf_Jv, lh_lf_Jv, 
                                    lh_th_Jv)),
                                 np.hstack((rh_ff_Jw, rh_mf_Jw, rh_rf_Jw, 
                                    rh_lf_Jw,
                                    lh_ff_Jw, lh_mf_Jw, lh_rf_Jw, lh_lf_Jw, 
                                    lh_th_Jw))]
        
        J = np.transpose(np.vstack((Jv, Jw)))
        Jpinv = np.linalg.pinv(J)
        print(f'\nJpinv:\n {Jpinv}\n')
        
        errp = ep(self.pdlast, ptips)
        err = np.concatenate((errp, np.zeros(2), errR, np.zeros(2)))
        
        qdlast = self.qd
        qddot = Jpinv @ (xddot + self.lam * err)
        print(f'\nqddot:\n {qddot}\n')
        print(f'\nself.qd:\n {self.qd}\n')

        self.qd += qddot * dt
        qd = self.qd
        self.pdlast = pd
        self.Rdlast = Rd

        return (qd, qddot, pd, vd, Rd, wd)

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
