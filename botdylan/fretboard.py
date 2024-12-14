import numpy as np

NUM_STRINGS = 6
# TODO: DETERMINE THESE:
GUITAR_HEIGHT = 0.020 # z-distance from the bottom of the guitar neck to the strings

# TODO: draw fretboard for guitar and map positions to chords
class Fretboard():
    def __init__(self, x0, y0, z0, dx, dy, num_frets):
        # Fretboard origin: the outer corner formed by the 1st fret and e_low
        self.x0 = x0        # xdistance to fretboard origin
        self.y0 = y0        # ydistance to fretboard origin
        self.z0 = z0        # string z height
        self.dx = dx       # distance between frets
        self.dy = dy        # distance between strings
        self.width = NUM_STRINGS * dy   # fretboard width between the E strings
        self.length = num_frets * dx    # fretboard length
        self.fretboard = np.ones((num_frets, NUM_STRINGS))
    def pf_from_chord(self, chord, p0):
        """
        Calculate the final position to move to to play a chord
        """
        # Get the (fret, string) positions of the chord for each finger, as well
        # as the list of fingers involved in playing the chord
        pf, playing_fingers = chord.placements()

        # Distinguish between the primary and secondary task positions based on
        # the fingers involved in playing this chord.
        primary_task_indeces = list(range(12))  # [d for d in range(12) if d % 3 == 2]
        secondary_task_indeces = []             # [d for d in range(12) if d % 3 < 2]
        for i in range(4):
            if i in playing_fingers:
                primary_task_indeces.extend([12+3*i+1, 12+3*i+2])
            else:
                secondary_task_indeces.append(12+3*i+2)

        # Incorporate a z-position (namely the height of the strings)
        pf = np.hstack((pf, self.z0 * np.ones((4,1))))
        # Set a desired clearance from the strings for fingers not involved in 
        # playing the chord
        string_clearance = 0.030

        # Estimate a desired wrist x-position as a mean of the finger x-positions:
        wrist_xd = self.x0 - self.dx * (np.nanmean(pf[:,0]) + 0.5)

        for i, finger_pf in enumerate(pf):
            # Identify the fingers that will be playing this chord:
            if not np.isnan(finger_pf[0]) and not np.isnan(finger_pf[1]):
                # Map the string and fret choordinates to xyz choordinates on the guitar
                finger_pf[0] = self.x0 - self.dx * (finger_pf[0] + 0.5)
                finger_pf[1] = self.y0 + self.dy * finger_pf[1]
            # The remaining fingers are not involved in playing the chord
            else:
                # ------------------DELETE!---------------------
                finger_pf[0] = wrist_xd + 0.033
                finger_pf[1] = p0[22]
                # Lift the finger some amount to not press the string
                finger_pf[2] += string_clearance # We can fine-tune this amount

        neck_base_z = self.z0 - GUITAR_HEIGHT # the z position of the bottom of the guitar
        lh_th_postion = np.array([[wrist_xd + 0.011, self.y0 + 2.5 * self.dy, neck_base_z]])
        primary_task_indeces.append(26)
        # secondary_task_indeces.extend([24, 25])

        pf = np.vstack((pf, lh_th_postion))
        return [np.concatenate((pf)), -wrist_xd, primary_task_indeces, secondary_task_indeces]
    def get_coord_from_pos(self, curr_pos):
        return (curr_pos[0] / self.dy, (curr_pos[1] - (self.dx / 2)) / self.dx)
    