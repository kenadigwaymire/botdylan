import numpy as np

NUM_STRINGS = 6
# TODO: DETERMINE THESE:
GUITAR_HEIGHT = 0.06 # z-distance from the bottom of the guitar neck to the strings

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
        # Get the (fret, string) positions of the chord for each finger
        pf = chord.placements()
        # Incorporate a z-position (namely the height of the strings)
        pf = np.hstack((pf, self.z0 * np.ones((4,1))))
        # Estimate a desired wrist x-position as a mean of the finger x-positions:
        wrist_xd = self.x0 - self.dx * (np.nanmean(pf[:,0]) + 0.5)

        # List of decent x and y offsets for each finger  relative to the wrist
        # (for the left hand) to guess a reasonable position for fingers that
        # aren't involved in playing the chord.
        finger_offset = [[-0.033, p0[13]],
                         [-0.011, p0[16]], 
                         [0.011, p0[19]], 
                         [0.033, p0[22]]]
        for i, finger_pf in enumerate(pf):
            # Identify the fingers that will be playing this chord:
            if not np.isnan(finger_pf[0]) and not np.isnan(finger_pf[1]):
                # Map the string and fret choordinates to xyz choordinates on the guitar
                finger_pf[0] = self.x0 - self.dx * (finger_pf[0] + 0.5)
                finger_pf[1] = self.y0 + self.dy * finger_pf[1]
            # The remaining fingers are not involved in playing the chord
            else:
                finger_pf[0] = wrist_xd + finger_offset[i][0]
                finger_pf[1] = finger_offset[i][1]
                # Lift the finger some amount to not press the string
                finger_pf[2] += 0.025 # We can fine-tune this amount

        neck_base_z = self.z0 - GUITAR_HEIGHT # the z position of the bottom of the guitar
        lh_th_postion = np.array([[wrist_xd - 0.033, p0[25], neck_base_z]])

        pf = np.vstack((pf, lh_th_postion))
        return [np.hstack((pf)), -wrist_xd]
    def get_coord_from_pos(self, curr_pos):
        return (curr_pos[0] / self.dy, (curr_pos[1] - (self.dx / 2)) / self.dx)
