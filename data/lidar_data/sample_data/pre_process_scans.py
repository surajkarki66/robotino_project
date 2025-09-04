import pandas as pd

# File paths
input_file = 'scans_flat.csv'  # <- updated
output_file = 'scans_clean.csv'

# Load CSV
df = pd.read_csv(input_file)

# Drop rows with any NaN or invalid values
df = df.dropna()

# Optional: remove rows with non-positive distances or negative intensities
df = df[(df['distance'] > 0) & (df['intensity'] >= 0)]

# Save cleaned CSV
df.to_csv(output_file, index=False)

print(f"Cleaned CSV saved as {output_file}. Total rows: {len(df)}")

