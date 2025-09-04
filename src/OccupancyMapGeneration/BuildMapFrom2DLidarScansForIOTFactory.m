%% CSV to Scans

% Load CSV
% CSV columns: distance | angle (in radians) | scan_index (0-based)
data = readmatrix('../../data/lidar_data/sample_data/scans_clean.csv');  

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

% Create a lidarSLAM object and set the map resolution and the max lidar range.

maxLidarRange = 8; %smaller than 10 meters
mapResolution = 20; % Set the max lidar range slightly smaller than the max scan range (8m), as the laser readings are less accurate near max range. Set the grid map resolution to 20 cells per meter, which gives a 5cm precision.
slamAlg = lidarSLAM(mapResolution, maxLidarRange);

% The following loop closure parameters are set empirically.
% Using higher loop closure threshold helps reject false positives in loop closure identification process.
% However, keep in mind that a high-score match may still be a bad match. 
% For example, scans collected in an environment that has similar or repeated features are more
% likely to produce false positives. Using a higher loop closure search radius allows the algorithm
% to search a wider range of the map around current pose estimate for loop closures.

slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

% Incrementally add scans to the slamAlg object. Scan numbers are printed if added to the map.
% The object rejects scans if the distance between scans is too small.
% Add the first 10 scans first to test your algorithm.
for i=1:10
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, iotScans{i});
    if isScanAccepted
        fprintf('Added scan %d \n', i);
    end
end

% Reconstruct the scene by plotting the scans and poses tracked by the slamAlg.
figure;
show(slamAlg);
title({'Map of the Environment','Pose Graph for Initial 10 Scans'});

% Observe the Effect of Loop Closures and the Optimization Process
% Continue to add scans in a loop. Loop closures should be automatically detected as the robot moves.
% Pose graph optimization is performed whenever a loop closure is identified.
% The output optimizationInfo has a field, IsPerformed, that indicates when pose graph optimization occurs.

firstTimeLCDetected = false;

figure;
for i=10:length(iotScans)
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, iotScans{i});
    if isScanAccepted
        fprintf('Added scan %d \n', i);
    end
    if ~isScanAccepted
        continue;
    end
    % visualize the first detected loop closure, if you want to see the
    % complete map building process, remove the if condition below
    if optimizationInfo.IsPerformed && ~firstTimeLCDetected
        show(slamAlg, 'Poses', 'off');
        hold on;
        show(slamAlg.PoseGraph); 
        hold off;
        firstTimeLCDetected = true;
        drawnow
    end
end
title('First loop closure');
% Visualize the Constructed Map and Trajectory of the Robot
% Plot the final built map after all scans are added to the slamAlg object.
% Though the previous for loop only plotted the initial closure, all the scans were added.

figure
show(slamAlg);
title({'Final Built Map of the Environment', 'Trajectory of the Robot'});

% Build Occupancy Grid Map
% The optimized scans and poses can be used to generate a occupancyMap,
% which represents the environment as a probabilistic occupancy grid.
[scans, optimizedPoses]  = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
figure; 
show(map);
hold on
show(slamAlg.PoseGraph, 'IDs', 'off');
hold off
title('Occupancy Grid Map Built Using Lidar SLAM');
% Saving Occupancy Map in .mat Format
save('../../data/maps/IOTFactoryOccupancyGridMap.mat', 'map');

load("../../data/maps/IOTFactoryOccupancyGridMap.mat","map")
show(map)

occMatrix = occupancyMatrix(map);

% Get dynamic thresholds from map object
occupied_thresh = map.OccupiedThreshold;
free_thresh = map.FreeThreshold;

% Create image
img = uint8(127 * ones(size(occMatrix)));  % Unknown = gray
img(occMatrix >= occupied_thresh) = 0;     % Occupied = black
img(occMatrix <= free_thresh) = 255;       % Free = white

% Save PGM image
imwrite(img, '../../data/maps/IOTFactoryOccupancyGridMap.pgm');

origin = map.GridLocationInWorld;          % [x, y] of bottom-left
resolution = 1 / map.Resolution;           % meters per cell

fid = fopen('../../data/maps/IOTFactoryOccupancyGridMap.yaml', 'w');
fprintf(fid, 'image: IOTFactoryOccupancyGridMap.pgm\n');
fprintf(fid, 'resolution: %.4f\n', resolution);
fprintf(fid, 'origin: [%.4f, %.4f, 0.0]\n', origin(1), origin(2));
fprintf(fid, 'negate: 0\n');
fprintf(fid, 'occupied_thresh: %.2f\n', occupied_thresh);
fprintf(fid, 'free_thresh: %.2f\n', free_thresh);
fclose(fid);
