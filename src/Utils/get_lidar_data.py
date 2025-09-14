import csv
from .rest_connection import OpenRobotinoAPI
import time

# Base URL and optional token
base_url = "http://172.21.21.90"
api_token = None  # Optional

# Create API client
robotino23 = OpenRobotinoAPI(base_url, api_token)

# CSV file path
csv_file = "lidar_scan_data.csv"

while True:
    scan_data = robotino23.get_scan0_data()
    if scan_data:
        # Create dictionary for this scan
        scan_dict = {
            "angles": scan_data.get("angles", []),
            "distances": scan_data.get("distances", []),
            "intensities": scan_data.get("intensities", [])
        }

        # Write dictionary as string in one row
        with open(csv_file, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([str(scan_dict)])  # one cell per row

    time.sleep(0.0)  # adjust sampling rate as needed
