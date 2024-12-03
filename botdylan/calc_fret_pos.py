def calculate_fret_offsets(neck_length, num_frets):
    # The ratio between adjacent frets
    fret_ratio = 0.943874
    
    # List to store the distances
    offsets = []
    
    # Calculate the offset from each fret
    for i in range(1, num_frets + 1):
        # The offset for the current fret
        offset = neck_length - neck_length * (fret_ratio ** i)
        offsets.append(offset)
    
    return offsets

# Parameters
neck_length = 2  # in units
num_frets = 24  # number of frets

# Calculate offsets
offsets = calculate_fret_offsets(neck_length, num_frets)

# Print the fret number and the corresponding offset
for i, offset in enumerate(offsets, start=1):
    print(f"Fret {i}: Offset = {offset:.4f} units")

