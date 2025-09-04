import csv
import re
import ast

# Input and output files
input_file = 'scans.csv'
output_file = 'scans_flat.csv'

# Regex to match bracketed lists
list_pattern = re.compile(r'\[.*?\]')

# Open output CSV and write header
with open(output_file, 'w', newline='') as f_out:
    writer = csv.writer(f_out)
    writer.writerow(['distance', 'angle', 'intensity', 'scan_index'])

# Process each scan
with open(input_file, 'r') as f_in:
    reader = csv.reader(f_in)
    
    for scan_idx, row in enumerate(reader):
        row_str = ' '.join(row)  # in case lists are in one column
        
        # Extract the three lists: angles, distances, intensities
        matches = list_pattern.findall(row_str)
        if len(matches) != 3:
            print(f"Skipping scan {scan_idx}, cannot parse lists")
            continue
        
        try:
            angles = ast.literal_eval(matches[0])
            distances = ast.literal_eval(matches[1])
            intensities = ast.literal_eval(matches[2])
        except Exception as e:
            print(f"Skipping scan {scan_idx}, invalid format: {e}")
            continue
        
        # Write all points with scan index
        with open(output_file, 'a', newline='') as f_out:
            writer = csv.writer(f_out)
            for d, a, i in zip(distances, angles, intensities):
                writer.writerow([d, a, i, scan_idx])

print(f"Flattened CSV saved as {output_file}")

