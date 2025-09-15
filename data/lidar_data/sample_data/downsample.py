import pandas as pd

# Load your LiDAR CSV (assuming space-separated)
df = pd.read_csv("iot_lidar_scans_1_clean.csv", delim_whitespace=True)  # columns: distance, angle, scan_index

# Step 1: Filter out unreasonable distances
min_distance = 0.05  # meters
max_distance = 10.0  # meters (adjust according to your LiDAR)
df = df[(df['distance'] > min_distance) & (df['distance'] < max_distance)]

# Step 2: Downsample each scan
downsample_factor = 2  # take every 2nd point; increase to 3 or 4 to be sparser
df_downsampled = df.groupby('scan_index').apply(lambda x: x.iloc[::downsample_factor]).reset_index(drop=True)

# Optional: save cleaned data
df_downsampled.to_csv("iot_lidar_scans_1_clean_downsampled.csv", index=False, sep=' ')
print(f"Original points: {len(df)}, After cleaning & downsampling: {len(df_downsampled)}")

