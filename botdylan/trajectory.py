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

# Grab the chords
from botdylan.chords             import *
from botdylan.fretboard          import *

# TODO: Flesh out a function to read the song that gets initialized when the tajectory
# gets initialized.
def song_info(song):
    # Eventually change these to be pulled or calculated from a song file
    T = 5
    chords = [G, C]
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
                            self.jointnames()[19:26])
        self.lh_mf = KinematicChain(node, 'world', 'lh_mftip',
                            self.jointnames()[19:22]+self.jointnames()[26:30])
        self.lh_rf = KinematicChain(node, 'world', 'lh_rftip', 
                            self.jointnames()[19:22]+self.jointnames()[30:34])
        self.lh_lf = KinematicChain(node, 'world', 'lh_lftip',
                            self.jointnames()[19:22]+self.jointnames()[34:39])
        self.lh_thumb = KinematicChain(node, 'world', 'lh_thtip',
                            self.jointnames()[19:22]+self.jointnames()[39:44])
        
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
            0.0, 0.0,                           # lh_WRJ2, lh_WRJ1
            0.021, 0.610, 0.526, 0.492,         # lh_FFJ4, lh_FFJ3, lh_FFJ2, lh_FFJ1
            0.021, 0.759, 0.781, 0.705,         # lh_MFJ4, lh_MFJ3, lh_MFJ2, lh_MFJ1
            0.002, 0.659, 0.849, 0.645,         # lh_RFJ4, lh_RFJ3, lh_RFJ2, lh_RFJ1
            0.075, 0.015, 0.630, 0.781, 0.594,  # lh_LFJ5, lh_LFJ4, lh_LFJ3, lh_LFJ2, lh_LFJ1
            0.277, 0.205, 0.110, 0.306, 0.10  # lh_THJ5, lh_THJ4, lh_THJ3, lh_THJ2, lh_THJ1
        ])
        
        self.qd = np.copy(self.q0)
        # Initial tip positions:
        self.p0 = self.get_ptips()

        # Other params
        self.lam = 40 # lambda for primary task
        self.lams = 10 # lambda for secondary task
        self.pdlast = np.copy(self.p0)
        
    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return [
            # len(jointnames) = 44
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
            "lh_THJ5", "lh_THJ4", "lh_THJ3", "lh_THJ2", "lh_THJ1" # thumb
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

                self.lh_ff.fkin(self.qd[19:26])[0],
                self.lh_mf.fkin(np.concatenate((self.qd[19:22],self.qd[26:30])))[0],
                self.lh_rf.fkin(np.concatenate((self.qd[19:22],self.qd[30:34])))[0],
                self.lh_lf.fkin(np.concatenate((self.qd[19:22],self.qd[34:39])))[0],
                self.lh_thumb.fkin(np.concatenate((self.qd[19:22],self.qd[39:44])))[0]
                ])
    

    def get_Jv(self):
        """
        Returns the translational Jacobian of size 27x44 for the robot (both hands).
        """
        Jv = np.zeros((27, 44))
        Jv[0:3, 0:6] = self.rh_ff.fkin(self.qd[0:6])[2]
        rh_mf_Jv = self.rh_mf.fkin(np.concatenate((self.qd[0:2],self.qd[6:10])))[2]
        Jv[3:6, 0:2], Jv[3:6, 6:10] = rh_mf_Jv[:,0:2], rh_mf_Jv[:,2:6]
        rh_rf_Jv = self.rh_rf.fkin(np.concatenate((self.qd[0:2],self.qd[10:14])))[2]
        Jv[6:9, 0:2], Jv[6:9, 10:14] = rh_rf_Jv[:,0:2], rh_rf_Jv[:,2:6]
        rh_lf_Jv = self.rh_lf.fkin(np.concatenate((self.qd[0:2],self.qd[14:19])))[2]
        Jv[9:12, 0:2], Jv[9:12, 14:19] = rh_lf_Jv[:,0:2], rh_lf_Jv[:,2:7]
        # We don't need the right-hand thumb — we set these to fixed joints
        # [rh_th_ptip, rh_th_Rtip, rh_th_Jv, rh_th_Jw] = self.rh_thumb.fkin(np.concatenate((self.qd[0:2],self.qd[19:24])))

        Jv[12:15, 19:26] = self.lh_ff.fkin(self.qd[19:26])[2]
        lh_mf_Jv = self.lh_mf.fkin(np.concatenate((self.qd[19:22],self.qd[26:30])))[2]
        Jv[15:18, 19:22], Jv[15:18, 26:30], = lh_mf_Jv[:,0:3], lh_mf_Jv[:,3:7]
        lh_rf_Jv = self.lh_rf.fkin(np.concatenate((self.qd[19:22],self.qd[30:34])))[2]
        Jv[18:21, 19:22], Jv[18:21, 30:34] = lh_rf_Jv[:,0:3], lh_rf_Jv[:,3:7]
        lh_lf_Jv = self.lh_lf.fkin(np.concatenate((self.qd[19:22],self.qd[34:39])))[2]
        Jv[21:24, 19:22], Jv[21:24, 34:39] = lh_lf_Jv[:,0:3], lh_lf_Jv[:,3:8]
        lh_th_Jv = self.lh_thumb.fkin(np.concatenate((self.qd[19:22],self.qd[39:44])))[2]
        Jv[24:27, 19:22], Jv[24:27, 39:44] = lh_th_Jv[:,0:3], lh_th_Jv[:,3:8]
        return Jv

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        # Initialize a guitar with: 20 frets, spaced 0.125 inches apart, and 
        # 6 strings spaced 0.0625 inches apart, at  a height of 0.2
        fretboard = Fretboard(x0=-0.080, y0=0.315, z0=0.09, dx=0.040, dy=0.005, num_frets=20)

        # Get the beat (T seconds), chords, and strumming pattern
        [T, chords, strumming_pattern] = song_info('some_song')

        if t <= T:
            prevChord = np.copy(self.p0)
            [nextChord, wrist_xd] = fretboard.pf_from_chord(G, self.p0, self.p0[0:12])
            # nextChord = np.copy(prevChord)
            # nextChord[2] -= 0.075
            print(f'\nprevChord:\n {prevChord[12:27]}\n')
            print(f'\nnextChord:\n {nextChord[12:27]}\n')
            print(f'\nwrist_xd:\n {wrist_xd}\n')
            (pd, vd) = goto(t, T, prevChord, nextChord)
            # pf = np.copy(self.p0[0:3])
            # pf[2] += 0.05
            # (pd, vd) = goto(t, T, self.p0[0:3], pf)
        elif t <= 2 * T:
            prevChord = fretboard.pf_from_chord(G, self.p0, self.p0[0:12])[0]
            [nextChord, wrist_xd] = fretboard.pf_from_chord(C, self.p0, self.p0[0:12])
            print(f'\nprevChord:\n {prevChord[12:27]}\n')
            print(f'\nnextChord:\n {nextChord[12:27]}\n')
            print(f'\nwrist_xd:\n {wrist_xd}\n')
            (pd, vd) = goto(t, T, prevChord, nextChord)
        else:
            vd = np.zeros(27)
            pd = self.get_ptips
        
        #print(f'\npd:\n {pd}\n')
        #print(f'\nvd:\n{vd}\n')
    
        # xddot = np.hstack((vd, wd))
        xddot = vd
        qd = np.copy(self.qd)

        Jv = self.get_Jv()

        ptips = self.get_ptips()
        print(f'\nptips:\n {ptips[12:27]}\n')

        #(ptip, Rtip, Jv, JW) = self.rh_ff.fkin(self.qd[0:6])

        # J = np.vstack((Jv, Jw))
        J = Jv
        #print(f"size of Jv: {Jv.shape}")
        # print(f'\nJv[0:12,:] - Right Hand:\n {Jv[0:12,:]}\n')
        # print(f'\nJv[14:27,:] - Left Hand:\n {Jv[12:27,:]}\n')
        Jt = np.transpose(J)
        Jpinv = np.linalg.pinv(J)
        # print(f'\nJpinv[:,0:20]:\n {Jpinv[:,0:20]}\n')
        # print(f'\nJpinv[:,20:44]:\n {Jpinv[:,20:44]}\n')

        #gamma = 0.00075
        # Jwinv = Jt @ (J @ Jt + gamma**2 * np.eye(J.shape[0]))
        # Jwinv[0:2,0:3] = np.zeros((2,3))
        # Jwinv[6:44,0:3] = np.zeros((34,3))
        # Jwinv[:,3:27] = np.zeros((44,24))
        #print(f'\nJwinv:\n {Jwinv}\n')
        
        errp = ep(self.pdlast, ptips)
        # err = np.concatenate((errp, errR))
        # errp = ep(self.pdlast[0:3], ptip)
        err = errp
        
        #qddot = np.zeros(44)
        qddot = Jpinv @ (xddot + self.lam * err)
        #qddot = Jt @ np.linalg.inv(J @ Jt) @ (xddot + self.lam * err)
        #print(f'\nqddot:\n {qddot}\n')
        #print(f'\nself.qd:\n {self.qd}\n')
        #print(f'\nxddot:\n {xddot}\n')
        #print(f'\nerror:\n {err}\n')
        # print(f"\nJ @ qddot:\n {J @ qddot - self.lam * err}")
        #print(f"\nJ @ qddot:\n {J @ qddot - self.lam * err}")

        # SECONDARY TASK: Push each joint toward humanlike hand position
        q_goal = np.copy(self.q0)   # We already initialize the hand in a human like position
        q_goal = np.copy(self.qd)
        q_goal[19] = wrist_xd       # "Comfortable" wrist position will vary depending on the chord
        q_goal[20] = 0
        q_goal[21] = 0
        # q_goal[22] = qd[-26]      
        q_goal[26] = 0
        q_goal[30] = 0
        # q_goal[34] = qd[30]
        q_goal[40] = np.copy(self.q0[40])
        q_goal[41] = 0

        print(f'\nq0:\n {self.q0[19:44]}\n')
        print(f'\nq_goal:\n {q_goal[19:44]}\n')
        
        W = np.array([
            1.0,
            1.0, 0.050, 
            0.75, 0.005, 0.005, 0.005, 
            0.75, 0.005, 0.005, 0.005, 
            0.75, 0.005, 0.005, 0.005, 
            0.25, 0.75, 0.005, 0.005, 0.005,
            0.005, 0.005, 0.005, 0.005, 0.005])
        W = np.concatenate((np.zeros(19), W))
        W = np.ones((44))
        qsdot = self.lams * np.diag(W) @ (q_goal - qd)

        # Combined joint velocity:
        qddot += (np.eye(J.shape[1]) - Jpinv @ J) @ qsdot

        qd += qddot * dt
        # print(f"\nqddot * dt:\n{qddot * dt}\n")
        self.qd = qd
        print(f"\nSelf.qd\n{self.qd[19:44]}\n")
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
