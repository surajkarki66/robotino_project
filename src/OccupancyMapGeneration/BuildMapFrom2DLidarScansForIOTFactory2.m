% Build Map from 2-D Lidar Scans Using SLAM

% Load Laser Scans
data = load("../../data/lidar_data/iot_factory_lidar_data_2_clean.mat");
scans = data.iotScans;

% Compute max distance directly from ranges
maxDist = 0;
for i = 1:length(scans)
    maxDist = max(maxDist, max(scans{i}.Ranges));
end

fprintf('Maximum distance in dataset: %.2f meters\n', maxDist);

% Create a lidarSLAM object and set the map resolution and the max lidar range.

maxLidarRange = 10;
mapResolution = 20; % Set the max lidar range slightly smaller than the max scan range (8m), as the laser readings are less accurate near max range. Set the grid map resolution to 20 cells per meter, which gives a 5cm precision.
slamAlg = lidarSLAM(mapResolution, maxLidarRange);

% The following loop closure parameters are set empirically.
% Using higher loop closure threshold helps reject false positives in loop closure identification process.
% However, keep in mind that a high-score match may still be a bad match. 
% For example, scans collected in an environment that has similar or repeated features are more
% likely to produce false positives. Using a higher loop closure search radius allows the algorithm
% to search a wider range of the map around current pose estimate for loop closures.

slamAlg.LoopClosureThreshold = 110;  
slamAlg.LoopClosureSearchRadius = 9;

% Observe the Effect of Loop Closures and the Optimization Process
% Continue to add scans in a loop. Loop closures should be automatically detected as the robot moves.
% Pose graph optimization is performed whenever a loop closure is identified.
% The output optimizationInfo has a field, IsPerformed, that indicates when pose graph optimization occurs.


figure;
for i=1:length(scans)
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{i});
    if isScanAccepted
       fprintf('Added scan %d \n', i);
    end
    if ~isScanAccepted
        continue;
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
title('Occupancy Map Built Using Lidar SLAM');

% Saving Occupancy Map in .mat Format
save('../../data/maps/IOTFactoryOccupancyMap5.mat', 'map');

load("../../data/maps/IOTFactoryOccupancyMap5.mat","map")
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
imwrite(img, '../../data/maps/IOTFactoryOccupancyMap5.pgm');

% Save Png image
imwrite(img, '../../data/maps/IOTFactoryOccupancyMap5.png');

origin = map.GridLocationInWorld;          % [x, y] of bottom-left
resolution = 1 / map.Resolution;           % meters per cell

fid = fopen('../../data/maps/IOTFactoryOccupancyMap5.yaml', 'w');
fprintf(fid, 'image: IOTFactoryOccupancyMap5.pgm\n');
fprintf(fid, 'resolution: %.4f\n', resolution);
fprintf(fid, 'origin: [%.4f, %.4f, 0.0]\n', origin(1), origin(2));
fprintf(fid, 'negate: 0\n');
fprintf(fid, 'occupied_thresh: %.2f\n', occupied_thresh);
fprintf(fid, 'free_thresh: %.2f\n', free_thresh);
fclose(fid);

% Flip occupancy matrix to match world coordinates
imgFlipped = flipud(img);

% Display map
figure;
imagesc(map.XWorldLimits, map.YWorldLimits, imgFlipped);
axis xy;  % y-axis upward
axis equal tight;
colormap(gray);
hold on;

% Plot robot trajectory in world coordinates
[~, optimizedPoses] = scansAndPoses(slamAlg);
plot(optimizedPoses(:,1), optimizedPoses(:,2), 'r-', 'LineWidth', 2);       % path
plot(optimizedPoses(end,1), optimizedPoses(end,2), 'ro', 'MarkerFaceColor','r'); % current robot

title('IOT Factory Occupancy Map with Robot Trajectory');
hold off;

% save PNG with trajectory overlay
frame = getframe(gcf);
imWithTrajectory = frame2im(frame);
imwrite(imWithTrajectory, '../../data/maps/IOTFactoryOccupancyMapwithTrajectory5.png');
