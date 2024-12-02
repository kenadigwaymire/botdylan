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

# TODO: DETERMINE THESE:
GUITAR_HEIGHT = 0.15 # z-distance from the bottom of the guitar neck to the strings

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
    def pd_from_chord(self, chord, p0):
        chord_position = np.copy(chord)
        chord_position = chord_position * np.array([self.dy, self.dx])
        neck_base_z = self.z - GUITAR_HEIGHT # the z position of the bottom of the guitar
        lh_th_postion = np.array([chord_position[0,0] - 0.001, p0[22], neck_base_z])
        chord_position[:, 1] += self.dx / 2
        chord_position = np.hstack((np.array(list(zip(chord_position[:,0], 
                                                     chord_position[:,1],
                                                     self.z * np.ones(4))))))
        chord_position = np.hstack((chord_position, lh_th_postion))
        print(f'\nchord pos:\n {chord_position}\n')
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
        self.rh_ff = KinematicChain(node, 'world', 'rh_fftip',
                            self.jointnames()[0:6])
        self.rh_mf = KinematicChain(node, 'world', 'rh_mftip',
                            self.jointnames()[0:2]+self.jointnames()[6:10])
        self.rh_rf = KinematicChain(node, 'world', 'rh_rftip', 
                            self.jointnames()[0:2]+self.jointnames()[10:14])
        self.rh_lf = KinematicChain(node, 'world', 'rh_lftip',
                            self.jointnames()[0:2]+self.jointnames()[14:19])
        # We don't need the right-hand thumb — we set these to fixed joints
        # self.rh_thumb = KinematicChain(node, 'world', 'rh_thtip',
        #                     self.jointnames()[0:2]+self.jointnames()[19:24])
        
        # LEFT HAND
        self.lh_ff = KinematicChain(node, 'world', 'lh_fftip',
                            self.jointnames()[19:26])
        self.lh_mf = KinematicChain(node, 'world', 'lh_mftip',
                            self.jointnames()[19:22]+self.jointnames()[26:30])
        self.lh_rf = KinematicChain(node, 'world', 'lh_rftip', 
                            self.jointnames()[19:22]+self.jointnames()[30:34])
        self.lh_lf = KinematicChain(node, 'world', 'lh_lftip',
                            self.jointnames()[19:22]+self.jointnames()[34:39])
        self.lh_thumb = KinematicChain(node, 'world', 'lh_thtip',
                            self.jointnames()[19:22]+[self.jointnames()[39]])
        
        # Init joint values (doesnt work rn cause chain is <6 and len(jointnames) = 48)
        # Initial joint positions:
        self.q0 = np.array([0.0, 0.0, 
                            0.0, 0.0, 0.0, 0.0, 
                            0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0,
                            0.0, 0.9599, 0.0, 0.0, 0.0,
                            0.2,
                            0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0,
                            0.0, 0.9599, 0.0, 0.0, 0.0,
                            1.5707963267948966])
        self.qd = np.copy(self.q0)
        # Initial tip positions:
        self.p0 = np.hstack([
            self.rh_ff.fkin(self.q0[0:6])[0],
            self.rh_mf.fkin(np.concatenate((self.q0[0:2],self.q0[6:10])))[0],
            self.rh_rf.fkin(np.concatenate((self.q0[0:2],self.q0[10:14])))[0],
            self.rh_lf.fkin(np.concatenate((self.q0[0:2],self.q0[14:19])))[0],

            self.lh_ff.fkin(self.q0[19:26])[0],
            self.lh_mf.fkin(np.concatenate((self.q0[19:22],self.q0[26:30])))[0],
            self.lh_rf.fkin(np.concatenate((self.q0[19:22],self.q0[30:34])))[0],
            self.lh_lf.fkin(np.concatenate((self.q0[19:22],self.q0[34:39])))[0],
            self.lh_thumb.fkin(np.concatenate((self.q0[19:22],self.q0[39:40])))[0]
            ])
        self.R0 = np.hstack([
            self.rh_ff.fkin(self.q0[0:6])[1],
            self.rh_mf.fkin(np.concatenate((self.q0[0:2],self.q0[6:10])))[1],
            self.rh_rf.fkin(np.concatenate((self.q0[0:2],self.q0[10:14])))[1],
            self.rh_lf.fkin(np.concatenate((self.q0[0:2],self.q0[14:19])))[1],

            self.lh_ff.fkin(self.q0[19:26])[1],
            self.lh_mf.fkin(np.concatenate((self.q0[19:22],self.q0[26:30])))[1],
            self.lh_rf.fkin(np.concatenate((self.q0[19:22],self.q0[30:34])))[1],
            self.lh_lf.fkin(np.concatenate((self.q0[19:22],self.q0[34:39])))[1],
            self.lh_thumb.fkin(np.concatenate((self.q0[19:22],self.q0[39:40])))[1]
            ])

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

        nextChord = fretboard.pd_from_chord(chords[0].get('G'), self.p0)
        nextChord = np.hstack((self.p0[0:12], nextChord))
        print(f'\nnextChord:\n {nextChord}\n')
        prevChord = np.copy(self.p0)
        (pd, vd) = goto(t, T, prevChord, nextChord)
        print(f'\nvd:\n {vd}\n')
        Rd = self.Rdlast # replace with rotation trajectory
        wd = np.zeros(27)
        xddot = np.hstack((vd, wd))

        qd = np.copy(self.qd)

        Jv = np.zeros((27, 40))
        Jw = np.zeros((27, 40))
        [rh_ff_ptip, rh_ff_Rtip, Jv[0:3, 0:6], Jw[0:3, 0:6]] = self.rh_ff.fkin(qd[0:6])
        [rh_mf_ptip, rh_mf_Rtip, rh_mf_Jv, rh_mf_Jw] = self.rh_mf.fkin(np.concatenate((qd[0:2],qd[6:10])))
        Jv[3:6, 0:2], Jv[3:6, 6:10], Jw[3:6, 0:2], Jw[3:6, 6:10] = rh_mf_Jv[:,0:2], rh_mf_Jv[:,2:6], rh_mf_Jw[:,0:2], rh_mf_Jw[:,2:6]
        [rh_rf_ptip, rh_rf_Rtip, rh_rf_Jv, rh_rf_Jw] = self.rh_rf.fkin(np.concatenate((qd[0:2],qd[10:14])))
        Jv[6:9, 0:2], Jv[6:9, 10:14], Jw[6:9, 0:2], Jw[6:9, 10:14] = rh_rf_Jv[:,0:2], rh_rf_Jv[:,2:6], rh_rf_Jw[:,0:2], rh_rf_Jw[:,2:6]
        [rh_lf_ptip, rh_lf_Rtip, rh_lf_Jv, rh_lf_Jw] = self.rh_lf.fkin(np.concatenate((qd[0:2],qd[14:19])))
        Jv[9:12, 0:2], Jv[9:12, 14:19], Jw[9:12, 0:2], Jw[9:12, 14:19] = rh_lf_Jv[:,0:2], rh_lf_Jv[:,2:7], rh_lf_Jw[:,0:2], rh_lf_Jw[:,2:7]
        # We don't need the right-hand thumb — we set these to fixed joints
        # [rh_th_ptip, rh_th_Rtip, rh_th_Jv, rh_th_Jw] = self.rh_thumb.fkin(np.concatenate((qd[0:2],qd[19:24])))

        [lh_ff_ptip, lh_ff_Rtip, Jv[12:15, 19:26], Jw[12:15, 19:26]] = self.lh_ff.fkin(qd[19:26])
        [lh_mf_ptip, lh_mf_Rtip, lh_mf_Jv, lh_mf_Jw] = self.lh_mf.fkin(np.concatenate((qd[19:22],qd[26:30])))
        Jv[15:18, 19:22], Jv[15:18, 26:30], Jw[15:18, 19:22], Jw[15:18, 26:30] = lh_mf_Jv[:,0:3], lh_mf_Jv[:,3:7], lh_mf_Jw[:,0:3], lh_mf_Jw[:,3:7]
        [lh_rf_ptip, lh_rf_Rtip, lh_rf_Jv, lh_rf_Jw] = self.lh_rf.fkin(np.concatenate((qd[19:22],qd[30:34])))
        Jv[18:21, 19:22], Jv[18:21, 30:34], Jw[18:21, 19:22], Jw[18:21, 30:34] = lh_rf_Jv[:,0:3], lh_rf_Jv[:,3:7], lh_rf_Jw[:,0:3], lh_rf_Jw[:,3:7]
        [lh_lf_ptip, lh_lf_Rtip, lh_lf_Jv, lh_lf_Jw] = self.lh_lf.fkin(np.concatenate((qd[19:22],qd[34:39])))
        Jv[21:24, 19:22], Jv[21:24, 34:39], Jw[21:24, 19:22], Jw[21:24, 34:39] = lh_lf_Jv[:,0:3], lh_lf_Jv[:,3:8], lh_lf_Jw[:,0:3], lh_lf_Jw[:,3:8]
        [lh_th_ptip, lh_th_Rtip, lh_th_Jv, lh_th_Jw] = self.lh_thumb.fkin(np.concatenate((qd[19:22],qd[39:40])))
        Jv[24:27, 19:22], Jv[24:27, 39:40], Jw[24:27, 19:22], Jw[24:27, 39:40] = lh_th_Jv[:,0:3], lh_th_Jv[:,3:4], lh_th_Jw[:,0:3], lh_th_Jw[:,3:4]

        # print(f'\nJv:\n {Jv}\n')
        # print(f'\nJw:\n {Jw}\n')
        # print(f'\nlh_lf_Jv:\n {lh_lf_Jv}\n')
        # print(f'\nlh_th_Jv:\n {lh_th_Jv}\n')

        [ptips, Rtips, errR] = [np.hstack((rh_ff_ptip, rh_mf_ptip, 
                                    rh_rf_ptip, rh_lf_ptip, 
                                    lh_ff_ptip, lh_mf_ptip, lh_rf_ptip, 
                                    lh_lf_ptip, lh_th_ptip)),
                                 np.hstack((rh_ff_Rtip, rh_mf_Rtip, 
                                    rh_rf_Rtip, rh_lf_Rtip, 
                                    lh_ff_Rtip, lh_mf_Rtip, lh_rf_Rtip, 
                                    lh_lf_Rtip, lh_th_Rtip)),
                                 np.hstack((eR(Rd[:,0:3], rh_ff_Rtip),
                                            eR(Rd[:,3:6], rh_mf_Rtip), 
                                            eR(Rd[:,6:9], rh_rf_Rtip), 
                                            eR(Rd[:,9:12], rh_lf_Rtip),
                                            eR(Rd[:,12:15], lh_ff_Rtip), 
                                            eR(Rd[:,15:18], lh_mf_Rtip), 
                                            eR(Rd[:,18:21], lh_rf_Rtip), 
                                            eR(Rd[:,21:24], lh_lf_Rtip), 
                                            eR(Rd[:,24:27], lh_th_Rtip)))]
        
        J = np.vstack((Jv, Jw))
        Jpinv = np.linalg.pinv(J)
        print(f'\nJpinv:\n {Jpinv}\n')
        
        errp = ep(self.pdlast, ptips)
        err = np.concatenate((errp, errR))
        
        qddot = Jpinv @ (xddot + self.lam * err)
        print(f'\nqddot:\n {qddot}\n')
        print(f'\nqddot.size:\n {qddot.shape}\n')
        print(f'\nself.qd:\n {self.qd}\n')
        print(f'\nxddot:\n {xddot}\n')
        print(f'\nself.qd:\n {self.lam * err}\n')

        qd += qddot * dt
        self.qd += qd
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
    print(f"\nTrajectory:\n{Trajectory}\n")
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
