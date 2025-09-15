%% Build Map from 2-D Lidar Scans Using SLAM

% Load pre-saved lidar scans
load('../../data/lidar_data/iot_factory_lidar_data_1_clean.mat', 'iotScans');

numScans = numel(iotScans);
disp(['Total scans loaded: ', num2str(numScans)]);

%% Create lidarSLAM object
maxLidarRange = 8;     % slightly smaller than max sensor range
mapResolution = 20;    % 20 cells per meter = 5cm precision
slamAlg = lidarSLAM(mapResolution, maxLidarRange);

% Loop closure parameters
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

%% Add first 10 scans (test)
for i = 1:min(10, numScans)
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, iotScans{i});
    if isScanAccepted
        fprintf('Added scan %d\n', i);
    end
end

figure;
show(slamAlg);
title({'Map of the Environment','Pose Graph for Initial 10 Scans'});

%% Observe effect of loop closures
firstTimeLCDetected = false;

figure;
for i = 11:numScans
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, iotScans{i});
    if ~isScanAccepted
        continue;
    end
    fprintf('Added scan %d\n', i);
    if optimizationInfo.IsPerformed && ~firstTimeLCDetected
        show(slamAlg, 'Poses', 'off');
        hold on;
        show(slamAlg.PoseGraph);
        hold off;
        firstTimeLCDetected = true;
        drawnow
    end
end
title('First Loop Closure');

%% Visualize final SLAM map + trajectory
figure;
show(slamAlg);
title({'Final Built Map of the Environment','Trajectory of the Robot'});

%% Build Occupancy Grid Map
[scans, optimizedPoses] = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);

figure; 
show(map);
hold on
show(slamAlg.PoseGraph, 'IDs', 'off');
hold off
title('Occupancy Map Built Using Lidar SLAM');

%% Save occupancy map
save('../../data/maps/IOTFactoryOccupancyMap.mat', 'map');

occMatrix = occupancyMatrix(map);

% Dynamic thresholds
occupied_thresh = map.OccupiedThreshold;
free_thresh = map.FreeThreshold;

% Create grayscale PGM-style image
img = uint8(127 * ones(size(occMatrix)));  % unknown = gray
img(occMatrix >= occupied_thresh) = 0;     % occupied = black
img(occMatrix <= free_thresh) = 255;       % free = white

% Save map images
imwrite(img, '../../data/maps/IOTFactoryOccupancyMap.pgm');
imwrite(img, '../../data/maps/IOTFactoryOccupancyMap.png');

% Save YAML metadata
origin = map.GridLocationInWorld;
resolution = 1 / map.Resolution;
fid = fopen('../../data/maps/IOTFactoryOccupancyMap.yaml', 'w');
fprintf(fid, 'image: IOTFactoryOccupancyMap.pgm\n');
fprintf(fid, 'resolution: %.4f\n', resolution);
fprintf(fid, 'origin: [%.4f, %.4f, 0.0]\n', origin(1), origin(2));
fprintf(fid, 'negate: 0\n');
fprintf(fid, 'occupied_thresh: %.2f\n', occupied_thresh);
fprintf(fid, 'free_thresh: %.2f\n', free_thresh);
fclose(fid);

%% Final Map with trajectory overlay
imgFlipped = flipud(img);
figure;
imagesc(map.XWorldLimits, map.YWorldLimits, imgFlipped);
axis xy; 
axis equal tight;
colormap(gray);
hold on;

plot(optimizedPoses(:,1), optimizedPoses(:,2), 'r-', 'LineWidth', 2);
plot(optimizedPoses(end,1), optimizedPoses(end,2), 'ro', 'MarkerFaceColor','r');
title('IOT Factory Occupancy Map with Robot Trajectory');
hold off;

% Save overlay map
frame = getframe(gcf);
imWithTrajectory = frame2im(frame);
imwrite(imWithTrajectory, '../../data/maps/IOTFactoryOccupancyMapwithTrajectory.png');

disp('✅ Final Occupancy Map and trajectory saved.');
