import numpy as np

def interpolate_string_positions(fret_positions, string_positions):
    """
    Interpolate the positions of each string between each fret pair.
    
    Parameters:
        fret_positions (dict): Fret names as keys and positions (x, y, z) as values.
        string_positions (dict): String names as keys and positions (x, y, z) as values.
    
    Returns:
        dict: A dictionary with string names as keys and lists of interpolated positions between frets.
    """
    string_fret_positions = {}

    # Remove 'neck' from fret_positions if it exists
    fret_positions = {k: v for k, v in fret_positions.items() if k != 'neck'}

    for str_name, str_pos in string_positions.items():
        # Get the fixed z position of the string (all strings share the same y)
        z_string = str_pos[2]
        
        # Initialize list to store the interpolated positions for this string
        desired_positions = {}

        # Make sure the fret names are correctly formatted and sortable
        sorted_frets = sorted(fret_positions.keys(), key=lambda x: extract_fret_number(x))

        for i in range(len(sorted_frets) - 1):
            fret_start_name = sorted_frets[i]
            fret_end_name = sorted_frets[i + 1]
            
            fret_start_pos = fret_positions[fret_start_name]
            fret_end_pos = fret_positions[fret_end_name]
            
            # Interpolate between the two frets (just between x positions)
            x_start = fret_start_pos[0]
            x_end = fret_end_pos[0]
            x_mid = (x_end + x_start)/2

            y_approx = (fret_end_pos[1] + str_pos[1])/2
            # Store
            desired_positions[f"{fret_end_name}"] = [x_mid, y_approx, z_string]

        string_fret_positions[str_name] = desired_positions

    return string_fret_positions


def extract_fret_number(fret_name):
    """ Extract the fret number from a fret name like 'fret0', 'fret1', etc. """
    try:
        # Extract the number after 'fret' and return it as an integer
        return int(fret_name.split('fret')[1])
    except (IndexError, ValueError):
        # If there's an issue with the format, handle it gracefully (e.g., return a large number)
        return float('inf')
