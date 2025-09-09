%% CSV to Scans

data = readmatrix('../../data/lidar_data/sample_data/scans_clean.csv');  
distances = data(:,1);
angles    = data(:,2);
scan_idx  = data(:,4);

unique_scans = unique(scan_idx);
numScans = numel(unique_scans);
disp(['Total scans: ', num2str(numScans)])

iotScans = cell(1, numScans);
for k = 1:numScans
    idx = scan_idx == unique_scans(k);
    ranges = distances(idx);
    angs   = angles(idx);
    iotScans{k} = lidarScan(ranges, angs);
end

%% SLAM setup
maxLidarRange = 8; 
mapResolution = 20; 
slamAlg = lidarSLAM(mapResolution, maxLidarRange);
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

%% GIF setup
gifFile = '../../data/maps/localization-map.gif';
delayTime = 0.3;
hFig = figure('Position', [100 100 800 800]);
firstTimeLCDetected = false;

%% Add scans and build GIF with robot path
for i = 1:length(iotScans)
    [isScanAccepted, ~, optimizationInfo] = addScan(slamAlg, iotScans{i});
    if ~isScanAccepted, continue; end
    fprintf('Added scan %d\n', i);
    
    % Build current occupancy map
    [scansNow, posesNow] = scansAndPoses(slamAlg);
    mapNow = buildMap(scansNow, posesNow, mapResolution, maxLidarRange);
    occNow = occupancyMatrix(mapNow);

    % Create PGM-style image
    occupied_thresh = mapNow.OccupiedThreshold;
    free_thresh = mapNow.FreeThreshold;
    imgNow = uint8(127 * ones(size(occNow)));
    imgNow(occNow >= occupied_thresh) = 0;
    imgNow(occNow <= free_thresh) = 255;
    
    % Display map
    imagesc(imgNow);
    colormap(gray);
    axis equal tight;
    hold on;
    
    % Plot robot trajectory
    if ~isempty(posesNow)
        xPos = posesNow(:,1);
        yPos = posesNow(:,2);
        % Convert world coordinates to image coordinates
        [rows, cols] = size(imgNow);
        xImg = round((xPos - mapNow.GridLocationInWorld(1)) / (1/mapResolution));
        yImg = round((yPos - mapNow.GridLocationInWorld(2)) / (1/mapResolution));
        yImg = rows - yImg; % invert y to match image coordinates
        plot(xImg, yImg, 'r-', 'LineWidth', 2);
        plot(xImg(end), yImg(end), 'ro', 'MarkerFaceColor','r'); % current robot
    end
    
    title(['Map + Robot Path: Scan ', num2str(i)]);
    hold off;
    drawnow;
    
    % Capture GIF frame
    frame = getframe(hFig);
    im = frame2im(frame);
    [imInd, cm] = rgb2ind(im, 256);
    if i == 1
        imwrite(imInd, cm, gifFile, 'gif', 'LoopCount', Inf, 'DelayTime', delayTime);
    else
        imwrite(imInd, cm, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
    end
    
    % Optional: visualize first loop closure
    if optimizationInfo.IsPerformed && ~firstTimeLCDetected
        hold on; show(slamAlg.PoseGraph); hold off;
        firstTimeLCDetected = true;
    end
end

disp(['✅ GIF with robot path saved: ', gifFile]);

%% Final map
[scans, optimizedPoses] = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);
occMatrix = occupancyMatrix(map);

% Final PGM with thresholds
occupied_thresh = map.OccupiedThreshold;
free_thresh = map.FreeThreshold;
img = uint8(127 * ones(size(occMatrix)));
img(occMatrix >= occupied_thresh) = 0;
img(occMatrix <= free_thresh) = 255;

% Visualize final map with trajectory
figure; imshow(img); hold on;
xPos = optimizedPoses(:,1);
yPos = optimizedPoses(:,2);
[rows, cols] = size(img);
xImg = round((xPos - map.GridLocationInWorld(1)) / (1/mapResolution));
yImg = round((yPos - map.GridLocationInWorld(2)) / (1/mapResolution));
yImg = rows - yImg;
plot(xImg, yImg, 'r-', 'LineWidth', 2);
plot(xImg(end), yImg(end), 'ro', 'MarkerFaceColor','r');
title('Final Occupancy Map + Robot Path');

% Save final PGM
pgmFile = '../../data/maps/localization-map.pgm';
imwrite(img, pgmFile);

% Save YAML
origin = map.GridLocationInWorld;
resolution = 1 / map.Resolution;
yamlFile = '../../data/maps/localization-map.yaml';
fid = fopen(yamlFile, 'w');
fprintf(fid, 'image: localization-map.pgm\n');
fprintf(fid, 'resolution: %.17g\n', resolution);
fprintf(fid, 'origin: [%.3f, %.3f, 0]\n', origin(1), origin(2));
fprintf(fid, 'negate: 0\n');
fprintf(fid, 'occupied_thresh: %.17g\n', occupied_thresh);
fprintf(fid, 'free_thresh: %.17g\n', free_thresh);
fclose(fid);

disp('✅ Final map with trajectory saved as PGM + YAML!');
