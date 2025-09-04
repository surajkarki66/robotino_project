import pandas as pd

# File path
input_file = 'scans_clean.csv'

# Load CSV
df = pd.read_csv(input_file)

# Check min and max values
for col in df.columns:
    min_val = df[col].min()
    max_val = df[col].max()
    print(f"{col}: min = {min_val}, max = {max_val}")

