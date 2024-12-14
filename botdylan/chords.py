import numpy as np

# Following guitar conventions
STRING_NOTES = {'e_high': 1, 'b': 2, 'g': 3, 'd': 4, 'a': 5, 'e_low': 6}

# Chord Generator
class Chords():
    """
    Define Chords as follows:

    chord = Chords('chordname', [[index_finger_fret, 'index_finger_string'],
                                [middle_finger_fret, 'middle_finger_string'],
                                [ring_finger_fret, 'ring_finger_string'],
                                [pinky_finger_fret, 'pinky_finger_string']])

    If a finger is not used to play a not, define both its finger_fret and 
    finger_string as None.

    The finger_fret is an integer from 1 to num_frets (currently 20), or None.
    By musical convention, the first fret is the closest to the end of the 
    guitar neck (furthest from the strumming hand).

    Make sure the 'finger_string' matches a key in STRING_NOTES, or is None.
    """
    def __init__(self, chordname, finger_placements):
        self.chordname = chordname
        self.finger_placements = finger_placements
    def name(self):
        return self.chordname
    def placements(self):
        """
        Remap the conventional chord & string positions to positions on the 
        strings and frets of our guitar, with its orientation.

        Also, return which fingers actually play the chord
            1: pointer
            2: middle
            3: ring
            4: pinky
        """
        playing_fingers = [] 
        placements = np.nan * np.ones((4,2))
        for i, finger_placement in enumerate(self.finger_placements):
            if not np.isnan(finger_placement[0]):
                playing_fingers.append(i)
                placements[i][0] = 20 - finger_placement[0]
                placements[i][1] = 6 - STRING_NOTES.get(finger_placement[1])
        return placements, playing_fingers


G = Chords('G', [[2, 'a'], [3, 'e_low'], [3, 'e_high'], [np.nan, np.nan]])
C = Chords('C', [[1, 'b'], [2, 'd'], [3, 'a'], [np.nan, np.nan]])
E = Chords('E', [[1, 'g'], [2, 'a'], [2, 'd'], [np.nan, np.nan]])
D = Chords('D', [[2, 'e_high'], [3, 'b'], [2, 'g'], [np.nan, np.nan]])
A = Chords('A', [[2, 'g'], [2, 'b'], [2, 'e_high'], [np.nan, np.nan]])
Am = Chords('Am', [[1, 'b'], [2, 'd'], [2, 'g'], [np.nan, np.nan]])
Em = Chords('Em', [[2, 'a'], [2, 'd'], [np.nan, np.nan], [np.nan, np.nan]])
F = Chords('F', [[1, 'e_high'], [1, 'b'], [2, 'g'], [3, 'd']])

