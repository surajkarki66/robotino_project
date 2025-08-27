%% CSV to wareHouseScans

% Load CSV
% CSV columns: distance | angle (in radians) | scan_index (0-based)
data = readmatrix('../../../sample_data/scans_clean.csv');  


distances = data(:,1);
angles    = data(:,2);
scan_idx  = data(:,4);

% Get unique scan indices
unique_scans = unique(scan_idx);
numScans = numel(unique_scans);

disp(numScans)

% Preallocate cell array for scans
iotScans = cell(1, numScans);

% Loop over each scan
for k = 1:numScans
    idx = scan_idx == unique_scans(k);  % points belonging to this scan
    ranges = distances(idx);
    angs   = angles(idx);
    
    % Create lidarScan object
    iotScans{k} = lidarScan(ranges, angs);
end

%% Display info
disp(['Total scans: ', num2str(numScans)]);
disp(iotScans{1});  % example: first scan