%% CSV to MAT conversion for Lidar Scans

% File paths
csvFile = '../../data/lidar_data/collected_data/iot_factory_lidar_data_3_clean.csv';
matFile = '../../data/lidar_data/iot_factory_lidar_data_3_clean.mat';

% Load CSV data
data = readmatrix(csvFile);
distances = data(:,1);
angles    = data(:,2);
scan_idx  = data(:,3);

% Identify unique scans
unique_scans = unique(scan_idx);
numScans = numel(unique_scans);

% Create lidarScan objects
iotScans = cell(1, numScans);
for k = 1:numScans
    idx = scan_idx == unique_scans(k);
    ranges = distances(idx);
    angs   = angles(idx);
    iotScans{k} = lidarScan(ranges, angs);
end

% Save all scans to a single .mat file
save(matFile, 'iotScans');

disp(['✅ All scans saved to ', matFile]);
