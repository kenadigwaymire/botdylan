import rclpy # type: ignore
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp, floor

# Grab the utilities
from botdylan.GeneratorNode      import GeneratorNode
from botdylan.TransformHelpers   import *
from botdylan.TrajectoryUtils    import *

# Grab the general fkin
from botdylan.KinematicChain     import KinematicChain

# Grab the guitar files
from botdylan.GuitarChain        import GuitarChain
from botdylan.FretPos            import *

# Grab the chords
from botdylan.chords             import *
from botdylan.fretboard          import *

# TODO: Flesh out a function to read the song that gets initialized when the tajectory
# gets initialized.
def song_info(song):
    """
    Returns details of the song.
    Parameters:
        song (str): Name or identifier of the song (currently unused).
    Returns:
        list: Contains tempo (T), a list of chords, and the strumming pattern.
    """
    
    T = 1
    chords = [G, C, E, G, E, C, G, C, E, G, E, C, G, C, E, G, E, C, G]
    strumming_pattern = "strum"
    return [T, chords, strumming_pattern]
       
#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        """
        Initializes the Trajectory object with kinematic chains, joint positions, and other parameters.
        Parameters:
            node (rclpy.node.Node): ROS2 node for communication.
        """
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
        self.lh_th = KinematicChain(node, 'world', 'lh_thtip',
                            self.jointnames()[19:23]+self.jointnames()[40:45])
        
        # Initial joint positions:
        self.q0 = np.array([
            # -------------------- Right Hand (STRUMMING) --------------------
            0.0, -0.30,                         # rh_WRJ2, rh_WRJ1
            0.157, 0.907, 0.323, 0.262,         # rh_FFJ4, rh_FFJ3, rh_FFJ2, rh_FFJ1
            0.017, 0.709, 0.206, 0.281,         # rh_MFJ4, rh_MFJ3, rh_MFJ2, rh_MFJ1
            0.032, 0.759, 0.249, 0.290,         # rh_RFJ4, rh_RFJ3, rh_RFJ2, rh_RFJ1
            0.075, 0.710, 0.630, 0.260, 0.330,  # rh_LFJ5, rh_LFJ4, rh_LFJ3, rh_LFJ2, rh_LFJ1
            # Right hand thumb fixed
            # -------------------- Left Hand (FRETTING) --------------------
            0.750,                              # right_hand_to_left_hand
            0.0, -0.300, 0.500,                 # lh_WRJ3, lh_WRJ2, lh_WRJ1
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
        self.lam = 25               # lambda for primary task
        self.lam2 = 5               # lambda for secondary task
        self.lam3 = 1             # lambda for tertiary task
        self.gamma = 0.000001       # gamma for weighted inverse
        self.pdlast = np.copy(self.p0)

        # # Initialize GuitarChain with fixed transformations from baseframe to frets
        # guitar_chain = GuitarChain(node, "world", "str_high_e")

        # # Get positions
        # fret_positions = guitar_chain.get_fret_positions()
        # string_positions = guitar_chain.get_string_positions()

        # # Print fret positions
        # for fret_name, fret_position in fret_positions.items():
        #     print(f"{fret_name}: {fret_position}")

        # # Print string positions
        # for str_name, str_pos in string_positions.items():
        #     print(f"{str_name}: {str_pos}")

        # string_fret_positions = interpolate_string_positions(fret_positions, string_positions)

        # # Print the interpolated positions
        # for str_name, positions in string_fret_positions.items():
        #     print(f"{str_name}:")
        #     for position_name, position in positions.items():
        #         print(f"  {position_name}: {position}")

        
    # Declare the joint names.
    def jointnames(self):
        """
        Returns a list of joint names based on the robot's URDF structure.
        Returns:
            list: Names of joints in order.
        """
        # Return a list of joint names FOR THE EXPECTED URDF!
        return [
            # len(jointnames) = 45
            # -------------------- Right Hand (STRUMMING) --------------------
            "rh_WRJ2", "rh_WRJ1",                                   # wrist 0:2
            "rh_FFJ4", "rh_FFJ3", "rh_FFJ2", "rh_FFJ1",             # pointer 2:6
            "rh_MFJ4", "rh_MFJ3", "rh_MFJ2", "rh_MFJ1",             # middle 6:10
            "rh_RFJ4", "rh_RFJ3", "rh_RFJ2", "rh_RFJ1",             # ring 10:14
            "rh_LFJ5", "rh_LFJ4", "rh_LFJ3", "rh_LFJ2", "rh_LFJ1",  # pinky 14:19
            # We won't be using the right hand thumb — we set these to fixed joints
            # "rh_THJ5", "rh_THJ4", "rh_THJ3", "rh_THJ2", "rh_THJ1",
            # -------------------- Left Hand (FRETTING) ----------------------
            "right_hand_to_left_hand",                              # prismatic joint for sliding along the neck 19
            "lh_WRJ3", "lh_WRJ2", "lh_WRJ1",                        # wrist 20:23
            "lh_FFJ4", "lh_FFJ3", "lh_FFJ2", "lh_FFJ1",             # pointer 23:27
            "lh_MFJ4", "lh_MFJ3", "lh_MFJ2", "lh_MFJ1",             # middle 27:31
            "lh_RFJ4", "lh_RFJ3", "lh_RFJ2", "lh_RFJ1",             # ring 31:35
            "lh_LFJ5", "lh_LFJ4", "lh_LFJ3", "lh_LFJ2", "lh_LFJ1",  # pinky 35:40
            "lh_THJ5", "lh_THJ4", "lh_THJ3", "lh_THJ2", "lh_THJ1"   # thumb 40:45
            ]


    def get_ptips(self):
        """
        Computes the 3D positions of finger tips for both hands.
        Returns:
            np.ndarray: Concatenated array of positions for all finger tips (length: 27).
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
                self.lh_th.fkin(np.concatenate((self.qd[19:23],self.qd[40:45])))[0]
                ])
    

    def get_Jv(self):
        """
        Computes the translational Jacobian for all fingers.
        Returns:
            np.ndarray: Jacobian matrix of size 27x45.
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
        lh_th_Jv = self.lh_th.fkin(np.concatenate((self.qd[19:23],self.qd[40:45])))[2]
        Jv[24:27, 19:23], Jv[24:27, 40:45] = lh_th_Jv[:,0:4], lh_th_Jv[:,4:9]
        return Jv
    
    def strumming_trajectory(self, t, T, fretboard, strum_pattern, strum_length, strum_depth):
        """
        Generates a strumming trajectory for the right hand based on time and strumming pattern.
        Parameters:
            t (float): Current time [s].
            T (float): Total time for a single strumming motion [s].
            strum_pattern (str): Type of strumming (e.g., "strum", "downstroke", "upstroke").
            strum_length (float): Length of strumming motion.
            strum_depth (float): Depth of strumming motion.
        Returns:
            tuple: Desired positions and velocities for the right-hand motion.
        """
        strum_pattern_list = ["strum", "downstroke", "upstroke"]
        y_mid = fretboard.y0 + 2.5 * fretboard.dy
        z0 = fretboard.z0

        rh_p0 = np.copy(self.p0[0:12])
        rh_pf = np.copy(self.p0[0:12])

        v0 = np.zeros_like(rh_p0)
        vf = np.zeros_like(rh_pf)

        if strum_pattern == "strum":
            t1 = fmod(t, T)
            if t1 < T/4:
                if t > T/4:
                    [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] = (y_mid - strum_length/2) * np.ones(4)
                    [rh_p0[2], rh_p0[5], rh_p0[8], rh_p0[11]] = (z0 + 0.0075) * np.ones(4)
                [rh_pf[1], rh_pf[4], rh_pf[7], rh_pf[10]] = y_mid * np.ones(4)
                [rh_pf[2], rh_pf[5], rh_pf[8], rh_pf[11]] = (z0 - strum_depth) * np.ones(4)
                [vf[1], vf[4], vf[7], vf[10]] = np.ones(4) * (strum_length / (T / 4))
            elif t1 < T/2:
                [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] = y_mid * np.ones(4)
                [rh_p0[2], rh_p0[5], rh_p0[8], rh_p0[11]] = (z0 - strum_depth) * np.ones(4)
                [rh_pf[1], rh_pf[4], rh_pf[7], rh_pf[10]] = (y_mid + strum_length/2) * np.ones(4)
                [rh_pf[2], rh_pf[5], rh_pf[8], rh_pf[11]] = (z0 + 0.0075) * np.ones(4)
                [v0[1], v0[4], v0[7], v0[10]] = np.ones(4) * (strum_length / (T / 4))
            elif t1 < 3*T/4:
                [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] = (y_mid + strum_length/2) * np.ones(4)
                [rh_p0[2], rh_p0[5], rh_p0[8], rh_p0[11]] = (z0 + 0.0075) * np.ones(4)
                [rh_pf[1], rh_pf[4], rh_pf[7], rh_pf[10]] = y_mid * np.ones(4)
                [rh_pf[2], rh_pf[5], rh_pf[8], rh_pf[11]] = (z0 - strum_depth) * np.ones(4)
                [vf[1], vf[4], vf[7], vf[10]] = -np.ones(4) * (strum_length / (T / 4))
            else:
                [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] = y_mid * np.ones(4)
                [rh_p0[2], rh_p0[5], rh_p0[8], rh_p0[11]] = (z0 - strum_depth) * np.ones(4)
                [rh_pf[1], rh_pf[4], rh_pf[7], rh_pf[10]] = (y_mid - strum_length/2) * np.ones(4)
                [rh_pf[2], rh_pf[5], rh_pf[8], rh_pf[11]] = (z0 + 0.0075) * np.ones(4)
                [v0[1], v0[4], v0[7], v0[10]] = -np.ones(4) * (strum_length / (T / 4))
            #(rh_pd, rh_vd) = goto(fmod(t1,T/4), T/4, rh_p0, rh_pf)
            (rh_pd, rh_vd) = spline(fmod(t1, T/4), T/4, rh_p0, rh_pf, v0, vf)

        elif strum_pattern == "downstroke":
            t1 = fmod(t, T)
            if t1 < T/3:
                if t > T/3:
                    [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] = (y_mid - strum_length/2) * np.ones(4)
                [rh_pf[1], rh_pf[4], rh_pf[7], rh_pf[10]] = y_mid * np.ones(4)
                [rh_pf[2], rh_pf[5], rh_pf[8], rh_pf[11]] = (z0 - strum_depth) * np.ones(4)
                [vf[1], vf[4], vf[7], vf[10]] = np.ones(4) * (strum_length / (T / 3))
            elif t1 < 2*T/3:
                [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] = y_mid * np.ones(4)
                [rh_p0[2], rh_p0[5], rh_p0[8], rh_p0[11]] = (z0 - strum_depth) * np.ones(4)
                [rh_pf[1], rh_pf[4], rh_pf[7], rh_pf[10]] = (y_mid + strum_length/2) * np.ones(4)
                [v0[1], v0[4], v0[7], v0[10]] = np.ones(4) * (strum_length / (T / 3))
            else:
                [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] = (y_mid + strum_length/2) * np.ones(4)
                [rh_pf[1], rh_pf[4], rh_pf[7], rh_pf[10]] = (y_mid - strum_length/2) * np.ones(4)
            #(rh_pd, rh_vd) = goto(fmod(t1,T/3), T/3, rh_p0, rh_pf)
            (rh_pd, rh_vd) = spline(fmod(t1, T/3), T/3, rh_p0, rh_pf, v0, vf)

        elif strum_pattern == "upstroke":
            t1 = fmod(t, T)
            if t1 < T/3:
                if t > T/3:
                    [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] = (y_mid - strum_length/2) * np.ones(4)
                [rh_pf[1], rh_pf[4], rh_pf[7], rh_pf[10]] = (y_mid + strum_length/2) * np.ones(4)
            elif t1 < 2*T/3:
                [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] = (y_mid + strum_length/2) * np.ones(4)
                [rh_pf[1], rh_pf[4], rh_pf[7], rh_pf[10]] = y_mid * np.ones(4)
                [rh_pf[2], rh_pf[5], rh_pf[8], rh_pf[11]] = (z0 - strum_depth) * np.ones(4)
                [vf[1], vf[4], vf[7], vf[10]] = -np.ones(4) * (strum_length / (T / 3))
            else:
                [rh_p0[1], rh_p0[4], rh_p0[7], rh_p0[10]] = y_mid * np.ones(4)
                [rh_p0[2], rh_p0[5], rh_p0[8], rh_p0[11]] = (z0 - strum_depth) * np.ones(4)
                [v0[1], v0[4], v0[7], v0[10]] = np.ones(4) * (strum_length / (T / 3))
            #(rh_pd, rh_vd) = goto(fmod(t1,T/3), T/3, rh_p0, rh_pf)
            (rh_pd, rh_vd) = spline(fmod(t1, T/3), T/3, rh_p0, rh_pf, v0, vf)

        else:
            raise ValueError(f"{strum_pattern} is not a valid strum pattern. "
                             f"Please refer to the following list of allowed strum patterns: {strum_pattern_list}")
        
        return (rh_pd, rh_vd)
    

    def fretting_trajectory(self, t, T, prevchord, nextchord):
        """
        Generates a fretting trajectory for the left hand.
        Parameters:
            t (float): Current time [s].
            T (float): Time to transition between chords [s].
            prevchord (np.ndarray): Previous chord position.
            nextchord (np.ndarray): Next chord position.
        Returns:
            tuple: Desired positions and velocities for the left-hand motion.
        """
        t2 = fmod(t, T)
        if t2 < T/2:
            # Move to the desired chord
            (lh_pd, lh_vd) = goto(t2, T/2, prevchord, nextchord)
        elif t2 < T:
            # Briefly hold the chord
            (lh_pd, lh_vd) = (nextchord, np.zeros(15))
        else:
            # Lift fingers off of the strings a bit before moving to next chord
            lifted_pos = np.copy(nextchord)
            for i in [2, 5, 8, 11]:
                lifted_pos[i] += 0.01
            # print(f"\nlifted_pos:\n{lifted_pos}\n")
            (lh_pd, lh_vd) = goto(fmod(t2, T/4), T/4, nextchord, lifted_pos)
        return lh_pd, lh_vd
    

    def task_dims(self, indcs):
        dims_dict = {0: "rh_ff_x", 1: "rh_ff_y", 2: "rh_ff_z",
                      3: "rh_mf_x", 4: "rh_mf_y", 5: "rh_mf_z",
                      6: "rh_rf_x", 7: "rh_rf_y", 8: "rh_rf_z",
                      9: "rh_lf_x", 10: "rh_lf_y", 11: "rh_lf_z",

                      12: "lh_ff_x", 13: "lh_ff_y", 14: "lh_ff_z",
                      15: "lh_mf_x", 16: "lh_mf_y", 17: "lh_mf_z",
                      18: "lh_rf_x", 19: "lh_rf_y", 20: "lh_rf_z",
                      21: "lh_lf_x", 22: "lh_lf_y", 23: "lh_lf_z",
                      24: "lh_th_x", 25: "lh_th_y", 26: "lh_th_z"}
        dims = []
        for i in indcs:
            dims.append(dims_dict.get(i))
        return dims


    # Evaluate at the given time. This was last called (dt) ago.
    def evaluate(self, t, dt):
        """
        Evaluates the current state of the trajectory at a given time.
        Parameters:
            t (float): Current time [s].
            dt (float): Time elapsed since last evaluation [s].
        Returns:
            tuple: Updated joint positions, joint velocities, desired positions, and velocities.
        """
        # Initialize a guitar with: 20 frets, spaced 0.125 inches apart, and 
        # 6 strings spaced 0.0625 inches apart, at  a height of 0.2
        fretboard = Fretboard(x0=-0.500, y0=0.250, z0=0.125, dx=0.050, dy=0.0075, num_frets=20)

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

        # Play the next chord until done
        if chord_ct < len(chords):
            [nextChord, wrist_xd, p_indeces, s_indeces] = fretboard.pf_from_chord(chords[chord_ct], self.p0)
            (rh_pd, rh_vd) = self.strumming_trajectory(t, T, fretboard, strumming_pattern, 12*fretboard.dy, .0025)
            (lh_pd, lh_vd) = self.fretting_trajectory(t, T, prevChord, nextChord)
            pd = np.concatenate((rh_pd, lh_pd))
            vd = np.concatenate((rh_vd, lh_vd))
        else:
            (pd, vd) = (np.copy(self.pdlast), np.zeros(27))
            wrist_xd = np.copy(self.qd[19])
            p_indeces = list(range(27))
            s_indeces = []

        qd = np.copy(self.qd)

        J = self.get_Jv()

        ptips = self.get_ptips()
        # print(f"size of J: {J.shape}")
        # print(f'\nJ[0:12,:] - Right Hand:\n {J[0:12,:]}\n')
        # print(f'\nJ[14:27,:] - Left Hand:\n {J[12:27,:]}\n')

        xddot_p = vd[p_indeces]
        J_p = J[p_indeces,:]
        Jt_p = np.transpose(J_p)
        Jwinv_p = Jt_p @ np.linalg.inv((J_p @ Jt_p + self.gamma**2 * np.eye(J_p.shape[0])))
        pdlast_p = self.pdlast[p_indeces]
        err_p = ep(pdlast_p, ptips[p_indeces])

        qddot = Jwinv_p @ (xddot_p + self.lam * err_p)
        # print(f'\nxddot_p:\n {xddot_p}\n')

        # SECONDARY TASK: for the left (fretting) hand, move the thumb to contact
        # the bottom of the guitar neck and lift any fingers that aren't involved
        # in playing the chord off of the guitar strings.
        xddot_s = vd[s_indeces]
        J_s = J[s_indeces,:]
        Jt_s = np.transpose(J_s)
        Jwinv_s = Jt_s @ np.linalg.inv((J_s @ Jt_s + self.gamma**2 * np.eye(J_s.shape[0])))
        pdlast_s = self.pdlast[s_indeces]
        err_s = ep(pdlast_s, ptips[s_indeces])

        # Debugging
        if chord_ct < len(chords):
            # print(f'\nprevChord:\n {prevChord}\n')
            # print(f'\nnextChord:\n {nextChord}\n')
            # print(f'\nptips:\n {ptips[p_indeces]}\n')
            # print(f'\nself.pdlast:\n {pdlast_p}\n')
            print(f'\nerr_p:\n {err_p}\n')
            # print(f'\nvd:\n {vd[p_indeces]}\n')
            # print(f'\nJ_p @ qddot:\n {J_p @ qddot}\n')
            # print(f'\nJ:\n {J}\n')
            # print(f'\nJ_p:\n {J_p}\n')
            # print(f'\nqddot:\n {qddot}\n')
            # print(f'\nerr_s:\n {err_s}\n')
            # print(f'\nwrist_xd:\n {wrist_xd}\n')
            # pdims = self.task_dims(p_indeces)
            # sdims = self.task_dims(s_indeces)
            # print(f'\npdims:\n {pdims}\n')
            # print(f'\nsdims:\n {sdims}\n')
            # print(f'\nxddot_p:\n {xddot_s}\n')

        qsdot = Jwinv_s @ (xddot_s + self.lam2 * err_s)
        qddot += (np.eye(J_p.shape[1]) - Jwinv_p @ J_p) @ qsdot

        # TERTIARY TASK: Push each joint toward humanlike hand position
        q_goal = np.copy(self.q0)   # We already initialize the hand in a human-like position
        q_goal[19] = wrist_xd       # "Comfortable" wrist position will vary depending on the chord

        # print(f'\nq_goal:\n {q_goal}\n')
        
        # Find the range of motion of each joint based on their min. and max. 
        # positions in a humanlike hand,all of which were fortunately given in 
        # the URDF — except the prismatic "right-to-left-hand" joint and lh_WRJ3
        # which we added in. We estimated the ranges ourselves for these 2.
        min_values = {
            "rh_WRJ2": -0.524, "rh_WRJ1": -0.698, 
            "rh_FFJ4": -0.349, "rh_FFJ3": -0.262, "rh_FFJ2": 0.000, "rh_FFJ1": 0.000,
            "rh_MFJ4": -0.349, "rh_MFJ3": -0.262, "rh_MFJ2": 0.000, "rh_MFJ1": 0.000,
            "rh_RFJ4": -0.349, "rh_RFJ3": -0.262, "rh_RFJ2": 0.000, "rh_RFJ1": 0.000,
            "rh_LFJ5": 0.000, "rh_LFJ4": -0.349, "rh_LFJ3": -0.262, "rh_LFJ2": 0.000, "rh_LFJ1": 0.000,
            "right_hand_to_left_hand": 0.200,
            "lh_WRJ3": -1.400, "lh_WRJ2": -0.524, "lh_WRJ1": -0.698, 
            "lh_FFJ4": -0.349, "lh_FFJ3": -0.262, "lh_FFJ2": 0.000, "lh_FFJ1": 0.000,
            "lh_MFJ4": -0.349, "lh_MFJ3": -0.262, "lh_MFJ2": 0.000, "lh_MFJ1": 0.000,
            "lh_RFJ4": -0.349, "lh_RFJ3": -0.262, "lh_RFJ2": 0.000, "lh_RFJ1": 0.000,
            "lh_LFJ5": 0.000, "lh_LFJ4": -0.349, "lh_LFJ3": -0.262, "lh_LFJ2": 0.000, "lh_LFJ1": 0.000,
            "lh_THJ5": -1.047, "lh_THJ4": 0.000, "lh_THJ3": -0.209, "lh_THJ2": -0.698, "lh_THJ1": -0.262
        }

        max_values = {
            "rh_WRJ2": 0.175, "rh_WRJ1": 0.489, 
            "rh_FFJ4": 0.349, "rh_FFJ3": 1.571, "rh_FFJ2": 1.571, "rh_FFJ1": 1.571,
            "rh_MFJ4": 0.349, "rh_MFJ3": 1.571, "rh_MFJ2": 1.571, "rh_MFJ1": 1.571,
            "rh_RFJ4": 0.349, "rh_RFJ3": 1.571, "rh_RFJ2": 1.571, "rh_RFJ1": 1.571,
            "rh_LFJ5": 0.785, "rh_LFJ4": 0.349, "rh_LFJ3": 1.571, "rh_LFJ2": 1.571, "rh_LFJ1": 1.571,
            "right_hand_to_left_hand": 1.000,
            "lh_WRJ3": 1.571, "lh_WRJ2": 0.175, "lh_WRJ1": 0.489, 
            "lh_FFJ4": 0.349, "lh_FFJ3": 1.571, "lh_FFJ2": 1.571, "lh_FFJ1": 1.571,
            "lh_MFJ4": 0.349, "lh_MFJ3": 1.571, "lh_MFJ2": 1.571, "lh_MFJ1": 1.571,
            "lh_RFJ4": 0.349, "lh_RFJ3": 1.571, "lh_RFJ2": 1.571, "lh_RFJ1": 1.571,
            "lh_LFJ5": 0.785, "lh_LFJ4": 0.349, "lh_LFJ3": 1.571, "lh_LFJ2": 1.571, "lh_LFJ1": 1.571,
            "lh_THJ5": 1.047, "lh_THJ4": 1.222, "lh_THJ3": 0.209, "lh_THJ2": 0.698, "lh_THJ1": 1.571
        }

        # Ordered list of keys (entire jointnames list)
        ordered_keys = self.jointnames()

        # Magnitudes / distances between max and min for each key
        W = np.array([abs(max_values[key] - min_values[key]) for key in ordered_keys])
        W[0:2] = np.zeros(2)
        # print("Weighted values for left hand:", W)

        qtdot = self.lam3 * np.diag(W) @ (q_goal - qd)

        # Combined joint velocity:
        qddot += (np.eye(J_s.shape[1]) - Jwinv_s @ J_s) @ qtdot

        qd += qddot * dt
        self.qd = qd
        self.pdlast = pd

        return (qd, qddot, pd, vd)

#
#  Main Code
#
def main(args=None):
    """
    Entry point of the script. Initializes ROS, sets up the trajectory generator, and runs the trajectory.
    Parameters:
        args (list, optional): Command-line arguments.
    """
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
