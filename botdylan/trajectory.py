import rclpy # type: ignore
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp, floor

# import sys
# print(sys.path) # To debug pathing

# Grab the utilities
from botdylan.GeneratorNode      import GeneratorNode
from botdylan.TransformHelpers   import *
from botdylan.TrajectoryUtils    import *

# Grab the general fkin
from botdylan.KinematicChain     import KinematicChain
from botdylan.GuitarChain        import GuitarChain

# Grab the chords
from botdylan.chords             import *
from botdylan.fretboard          import *

# TODO: Flesh out a function to read the song that gets initialized when the tajectory
# gets initialized.
def song_info(song):
    # Eventually change these to be pulled or calculated from a song file
    T = 1
    chords = [G, C, E, G, E, C, G]
    strumming_pattern = []
    return [T, chords, strumming_pattern]
       
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
                            self.jointnames()[19:27])
        self.lh_mf = KinematicChain(node, 'world', 'lh_mftip',
                            self.jointnames()[19:23]+self.jointnames()[27:31])
        self.lh_rf = KinematicChain(node, 'world', 'lh_rftip', 
                            self.jointnames()[19:23]+self.jointnames()[31:35])
        self.lh_lf = KinematicChain(node, 'world', 'lh_lftip',
                            self.jointnames()[19:23]+self.jointnames()[35:40])
        self.lh_thumb = KinematicChain(node, 'world', 'lh_thtip',
                            self.jointnames()[19:23]+self.jointnames()[40:45])
        
        # Initial joint positions:
        self.q0 = np.array([
            # -------------------- Right Hand (STRUMMING) --------------------
            0.0, 0.0,                           # rh_WRJ2, rh_WRJ1
            0.157, 0.907, 0.968, 1.036,         # rh_FFJ4, rh_FFJ3, rh_FFJ2, rh_FFJ1
            0.017, 0.709, 0.620, 0.959,         # rh_MFJ4, rh_MFJ3, rh_MFJ2, rh_MFJ1
            0.032, 0.759, 0.747, 0.951,         # rh_RFJ4, rh_RFJ3, rh_RFJ2, rh_RFJ1
            0.075, 0.015, 0.630, 0.781, 0.594,  # rh_LFJ5, rh_LFJ4, rh_LFJ3, rh_LFJ2, rh_LFJ1
            # Right hand thumb fixed
            # -------------------- Left Hand (FRETTING) --------------------
            0.750,                              # right_hand_to_left_hand
            0.0, -0.300, 0.100,                 # lh_WRJ3, lh_WRJ2, lh_WRJ1
            -0.175, 0.600, 0.525, 0.500,        # lh_FFJ4, lh_FFJ3, lh_FFJ2, lh_FFJ1
            -0.050, 0.615, 0.650, 0.425,        # lh_MFJ4, lh_MFJ3, lh_MFJ2, lh_MFJ1
            -0.050, 0.525, 0.900, 0.415,        # lh_RFJ4, lh_RFJ3, lh_RFJ2, lh_RFJ1
            0.125, -0.225, 0.640, 0.840, 0.500, # lh_LFJ5, lh_LFJ4, lh_LFJ3, lh_LFJ2, lh_LFJ1
            -0.525, 0.205, 0.110, 0.366, 0.265  # lh_THJ5, lh_THJ4, lh_THJ3, lh_THJ2, lh_THJ1
        ])
        
        self.qd = np.copy(self.q0)
        # Initial tip positions:
        self.p0 = self.get_ptips()

        # Other params
        self.lam = 30           # lambda for primary task
        self.lams = 5           # lambda for secondary task
        self.gamma = 0.075      # gamma for weighted inverse
        self.pdlast = np.copy(self.p0)

        # Initialize KinematicChain with fixed transformations
        str_pos = GuitarChain(node, "world", "str_high_e")

        # Compute the forward kinematics
        tip_position, tip_rotation = str_pos.fkin()

        print(tip_position)
        # Use the tip position and rotation for further calculations or visualization
        node.get_logger().info(f"Tip Position: {tip_position}")
        node.get_logger().info(f"Tip Rotation: {tip_rotation}")
        
    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return [
            # len(jointnames) = 45
            # -------------------- Right Hand (STRUMMING) --------------------
            "rh_WRJ2", "rh_WRJ1",                                   # wrist
            "rh_FFJ4", "rh_FFJ3", "rh_FFJ2", "rh_FFJ1",             # pointer
            "rh_MFJ4", "rh_MFJ3", "rh_MFJ2", "rh_MFJ1",             # middle
            "rh_RFJ4", "rh_RFJ3", "rh_RFJ2", "rh_RFJ1",             # ring
            "rh_LFJ5", "rh_LFJ4", "rh_LFJ3", "rh_LFJ2", "rh_LFJ1",  # pinky
            # We won't be using the right hand thumb — we set these to fixed joints
            # "rh_THJ5", "rh_THJ4", "rh_THJ3", "rh_THJ2", "rh_THJ1",
            # -------------------- Left Hand (FRETTING) ----------------------
            "right_hand_to_left_hand",                              # prismatic joint for sliding along the neck
            "lh_WRJ3", "lh_WRJ2", "lh_WRJ1",                        # wrist
            "lh_FFJ4", "lh_FFJ3", "lh_FFJ2", "lh_FFJ1",             # pointer
            "lh_MFJ4", "lh_MFJ3", "lh_MFJ2", "lh_MFJ1",             # middle
            "lh_RFJ4", "lh_RFJ3", "lh_RFJ2", "lh_RFJ1",             # ring
            "lh_LFJ5", "lh_LFJ4", "lh_LFJ3", "lh_LFJ2", "lh_LFJ1",  # pinky
            "lh_THJ5", "lh_THJ4", "lh_THJ3", "lh_THJ2", "lh_THJ1"   # thumb
            ]


    def get_ptips(self):
        """
        Returns the positions of the fingers as an array of length 27, representing
        the x, y, and z positions of each of the 9 fingers used (in the order that
        they appear in jointnames).
        """
        return np.concatenate([
                self.rh_ff.fkin(self.qd[0:6])[0],
                self.rh_mf.fkin(np.concatenate((self.qd[0:2],self.qd[6:10])))[0],
                self.rh_rf.fkin(np.concatenate((self.qd[0:2],self.qd[10:14])))[0],
                self.rh_lf.fkin(np.concatenate((self.qd[0:2],self.qd[14:19])))[0],

                self.lh_ff.fkin(self.qd[19:27])[0],
                self.lh_mf.fkin(np.concatenate((self.qd[19:23],self.qd[27:31])))[0],
                self.lh_rf.fkin(np.concatenate((self.qd[19:23],self.qd[31:35])))[0],
                self.lh_lf.fkin(np.concatenate((self.qd[19:23],self.qd[35:40])))[0],
                self.lh_thumb.fkin(np.concatenate((self.qd[19:23],self.qd[40:45])))[0]
                ])
    

    def get_Jv(self):
        """
        Returns the translational Jacobian of size 27x45 for the robot (both hands).
        """
        Jv = np.zeros((27, 45))
        Jv[0:3, 0:6] = self.rh_ff.fkin(self.qd[0:6])[2]
        rh_mf_Jv = self.rh_mf.fkin(np.concatenate((self.qd[0:2],self.qd[6:10])))[2]
        Jv[3:6, 0:2], Jv[3:6, 6:10] = rh_mf_Jv[:,0:2], rh_mf_Jv[:,2:6]
        rh_rf_Jv = self.rh_rf.fkin(np.concatenate((self.qd[0:2],self.qd[10:14])))[2]
        Jv[6:9, 0:2], Jv[6:9, 10:14] = rh_rf_Jv[:,0:2], rh_rf_Jv[:,2:6]
        rh_lf_Jv = self.rh_lf.fkin(np.concatenate((self.qd[0:2],self.qd[14:19])))[2]
        Jv[9:12, 0:2], Jv[9:12, 14:19] = rh_lf_Jv[:,0:2], rh_lf_Jv[:,2:7]
        # We don't need the right-hand thumb — we set these to fixed joints
        # [rh_th_ptip, rh_th_Rtip, rh_th_Jv, rh_th_Jw] = self.rh_thumb.fkin(np.concatenate((self.qd[0:2],self.qd[19:24])))

        Jv[12:15, 19:27] = self.lh_ff.fkin(self.qd[19:27])[2]
        lh_mf_Jv = self.lh_mf.fkin(np.concatenate((self.qd[19:23],self.qd[27:31])))[2]
        Jv[15:18, 19:23], Jv[15:18, 27:31], = lh_mf_Jv[:,0:4], lh_mf_Jv[:,4:8]
        lh_rf_Jv = self.lh_rf.fkin(np.concatenate((self.qd[19:23],self.qd[31:35])))[2]
        Jv[18:21, 19:23], Jv[18:21, 31:35] = lh_rf_Jv[:,0:4], lh_rf_Jv[:,4:8]
        lh_lf_Jv = self.lh_lf.fkin(np.concatenate((self.qd[19:23],self.qd[35:40])))[2]
        Jv[21:24, 19:23], Jv[21:24, 35:40] = lh_lf_Jv[:,0:4], lh_lf_Jv[:,4:9]
        lh_th_Jv = self.lh_thumb.fkin(np.concatenate((self.qd[19:23],self.qd[40:45])))[2]
        Jv[24:27, 19:23], Jv[24:27, 40:45] = lh_th_Jv[:,0:4], lh_th_Jv[:,4:9]
        return Jv
    

    def strumming_trajectory(self, t, T, strum_pattern, strum_length, strum_depth):
        strum_pattern_list = ["strum", "downstroke", "upstroke"]

        rh_p0 = np.copy(self.p0[0:12])
        rh_pf = np.copy(self.p0[0:12])
        if strum_pattern == "strum":
            t1 = fmod(t, T/4)
            if t1 < T/4:
                [rh_pf[1], rh_pf[4], rh_pf[7], rh_pf[10]] = [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] + strum_length/2 * np.ones(4)
                [rh_pf[2], rh_pf[5], rh_pf[8], rh_pf[11]] = [rh_p0[2], rh_p0[5], rh_p0[8], rh_p0[11]] - strum_depth/2 * np.ones(4)
            elif t1 < T/2:
                [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] = [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] + strum_length/2 * np.ones(4)
                [rh_p0[2], rh_p0[5], rh_p0[8], rh_p0[11]] = [rh_p0[2], rh_p0[5], rh_p0[8], rh_p0[11]] - strum_depth/2 * np.ones(4)
                [rh_pf[1], rh_pf[4], rh_pf[7], rh_pf[10]] = [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] + strum_length * np.ones(4)
            elif t1 < 3*T/4:
                [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] = [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] + strum_length * np.ones(4)
                [rh_pf[1], rh_pf[4], rh_pf[7], rh_pf[10]] = [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] + strum_length/2 * np.ones(4)
                [rh_pf[2], rh_pf[5], rh_pf[8], rh_pf[11]] = [rh_p0[2], rh_p0[5], rh_p0[8], rh_p0[11]] - strum_depth/2 * np.ones(4)
            else:
                [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] = [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] + strum_length/2 * np.ones(4)
                [rh_p0[2], rh_p0[5], rh_p0[8], rh_p0[11]] = [rh_p0[2], rh_p0[5], rh_p0[8], rh_p0[11]] - strum_depth/2 * np.ones(4)
            (rh_pd, rh_vd) = goto(fmod(t1,T/4), T/4, rh_p0, rh_pf)

        elif strum_pattern == "downstroke":
            t1 = fmod(t, T/3)
            if t1 < T/3:
                [rh_pf[1], rh_pf[4], rh_pf[7], rh_pf[10]] = [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] + strum_length/2 * np.ones(4)
                [rh_pf[2], rh_pf[5], rh_pf[8], rh_pf[11]] = [rh_p0[2], rh_p0[5], rh_p0[8], rh_p0[11]] - strum_depth/2 * np.ones(4)
            elif t1 < 2*T/3:
                [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] = [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] + strum_length/2 * np.ones(4)
                [rh_p0[2], rh_p0[5], rh_p0[8], rh_p0[11]] = [rh_p0[2], rh_p0[5], rh_p0[8], rh_p0[11]] - strum_depth/2 * np.ones(4)
                [rh_pf[1], rh_pf[4], rh_pf[7], rh_pf[10]] = [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] + strum_length * np.ones(4)
            else:
                [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] = [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] + strum_length * np.ones(4)
            (rh_pd, rh_vd) = goto(fmod(t1,T/3), T/3, rh_p0, rh_pf)

        elif strum_pattern == "upstroke":
            t1 = fmod(t, T/3)
            if t1 < T/3:
                [rh_pf[1], rh_pf[4], rh_pf[7], rh_pf[10]] = [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] + strum_length * np.ones(4)
            elif t1 < 2*T/3:
                [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] = [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] + strum_length * np.ones(4)
                [rh_pf[1], rh_pf[4], rh_pf[7], rh_pf[10]] = [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] + strum_length/2 * np.ones(4)
                [rh_pf[2], rh_pf[5], rh_pf[8], rh_pf[11]] = [rh_p0[2], rh_p0[5], rh_p0[8], rh_p0[11]] - strum_depth/2 * np.ones(4)
            else:
                [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] = [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] + strum_length/2 * np.ones(4)
                [rh_p0[2], rh_p0[5], rh_p0[8], rh_p0[11]] = [rh_p0[2], rh_p0[5], rh_p0[8], rh_p0[11]] - strum_depth/2 * np.ones(4)
            (rh_pd, rh_vd) = goto(fmod(t1,T/3), T/3, rh_p0, rh_pf)

        else:
            raise ValueError(f"{strum_pattern} is not a valid strum pattern. "
                             f"Please refer to the following list of allowed strum patterns: {strum_pattern_list}")
        
        return (rh_pd, rh_vd)
    

    def fretting_trajectory(self, t, T, prevchord, nextchord):
        t2 = fmod(t, T)
        if t2 < T/2:
            # Move to the desired chord
            (lh_pd, lh_vd) = goto(t2, T/2, prevchord, nextchord)
        elif t2 < 3*T/4:
            # Briefly hold the chord
            (lh_pd, lh_vd) = (nextchord, np.zeros(15))
        else:
            # Lift fingers off of the strings a bit before moving to next chord
            lifted_pos = np.copy(nextchord)
            for i in [2, 5, 8, 11]:
                lifted_pos[i] += 0.01
            print(f"\nnextchord:\n{nextchord}\n")
            print(f"\nlifted_pos:\n{lifted_pos}\n")
            (lh_pd, lh_vd) = goto(fmod(t2, T/4), T/4, nextchord, lifted_pos)
        return lh_pd, lh_vd


    # Evaluate at the given time. This was last called (dt) ago.
    def evaluate(self, t, dt):
        # Initialize a guitar with: 20 frets, spaced 0.125 inches apart, and 
        # 6 strings spaced 0.0625 inches apart, at  a height of 0.2
        fretboard = Fretboard(x0=-0.080, y0=0.315, z0=0.09, dx=0.050, dy=0.0143, num_frets=20)

        # Get the beat (T seconds), chords, and strumming pattern
        [T, chords, strumming_pattern] = song_info('some_song')
        # Iterate through the chords to play
        if floor(t/T) <= len(chords):
            chord_ct = floor(t/T)
        else:
            chord_ct = len(chords)
        # print(f"\nChord:\n{chords[chord_ct]}\n")

        # Determine if starting from p0 or a chord that's already been played
        if chord_ct - 1 < 0:
            prevChord = np.copy(self.p0[12:27])
        else:
            prevChord = fretboard.pf_from_chord(chords[chord_ct-1], self.p0)[0]
            # # No need to lift the fingers when the song is over.
            # if chord_ct != len(chords):
            #     for i in [2, 5, 8, 11]:
            #         prevChord[i] += 0.01

        # Play the next chord. Return to p0 when the song is done
        if chord_ct < len(chords):
            [nextChord, wrist_xd] = fretboard.pf_from_chord(chords[chord_ct], self.p0)
            (lh_pd, lh_vd) = self.fretting_trajectory(t, T, prevChord, nextChord)
        else:
            (lh_pd, lh_vd) = (np.copy(p0), np.zeros(15))
            wrist_xd = np.copy(self.q0[19])

        (rh_pd, rh_vd) = (self.p0[0:12], np.zeros(12))
        pd = np.concatenate((rh_pd, lh_pd))
        vd = np.concatenate((rh_vd, lh_vd))

        print(f'\nprevChord:\n {prevChord}\n')
        print(f'\nnextChord:\n {nextChord}\n')
        print(f'\nwrist_xd:\n {wrist_xd}\n')
        xddot = vd
        qd = np.copy(self.qd)

        Jv = self.get_Jv()

        ptips = self.get_ptips()
        print(f'\nptips:\n {ptips[12:27]}\n')

        J = Jv
        # print(f"size of Jv: {Jv.shape}")
        # print(f'\nJv[0:12,:] - Right Hand:\n {Jv[0:12,:]}\n')
        # print(f'\nJv[14:27,:] - Left Hand:\n {Jv[12:27,:]}\n')
        Jt = np.transpose(J)
        Jwinv = Jt @ np.linalg.inv((J @ Jt + self.gamma**2 * np.eye(J.shape[0])))
        # print(f'\nJwinv:\n {Jwinv}\n')
        
        err = ep(self.pdlast, ptips)
        
        qddot = Jwinv @ (xddot + self.lam * err)
        #qddot = Jt @ np.linalg.inv(J @ Jt) @ (xddot + self.lam * err)
        #print(f'\nqddot:\n {qddot}\n')
        #print(f'\nself.qd:\n {self.qd}\n')
        #print(f'\nxddot:\n {xddot}\n')
        print(f'\nerror:\n {err}\n')

        # SECONDARY TASK: Push each joint toward humanlike hand position
        q_goal = np.copy(self.q0)   # We already initialize the hand in a human-like position
        q_goal[19] = wrist_xd       # "Comfortable" wrist position will vary depending on the chord
        # q_goal[20] = 0
        # q_goal[21] = 0
        # # q_goal[23] = qd[-26]      
        # q_goal[26] = 0
        # q_goal[30] = 0
        # # q_goal[34] = qd[30]
        # q_goal[40] = np.copy(self.q0[40])
        # q_goal[41] = 0

        print(f'\nq0:\n {self.q0[19:45]}\n')
        print(f'\nq_goal:\n {q_goal[19:45]}\n')
        
        W = np.array([
            1.0,
            0.50, 1.0, 0.050, 
            0.75, 0.005, 0.005, 0.005, 
            0.75, 0.005, 0.005, 0.005, 
            0.75, 0.005, 0.005, 0.005, 
            0.25, 0.75, 0.005, 0.005, 0.005,
            0.005, 0.005, 0.005, 0.005, 0.005])
        W = np.concatenate((np.zeros(19), W))
        W = np.ones((45))
        qsdot = self.lams * np.diag(W) @ (q_goal - qd)

        # Combined joint velocity:
        qddot += (np.eye(J.shape[1]) - Jwinv @ J) @ qsdot

        qd += qddot * dt
        # print(f"\nqddot * dt:\n{qddot * dt}\n")
        self.qd = qd
        print(f"\nSelf.qd\n{self.qd[19:45]}\n")
        self.pdlast = pd

        return (qd, qddot, pd, vd)

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
