%% Load Laser Scans
data = load("../../data/lidar_data/iot_factory_lidar_data_1_clean.mat");
scans = data.iotScans;  % cell array of lidarScan objects
numScans = numel(scans);
fprintf('Total scans loaded: %d\n', numScans);

%% Compute maxLidarRange dynamically
maxDist = 0;
for i = 1:numScans
    maxDist = max(maxDist, max(scans{i}.Ranges));
end
maxLidarRange = maxDist * 0.95;  % slightly smaller than max distance
fprintf('Maximum distance in scans: %.2f meters\n', maxDist);
fprintf('Max lidar range set to: %.2f meters\n', maxLidarRange);

%% SLAM setup
mapResolution = 20; 
slamAlg = lidarSLAM(mapResolution, maxLidarRange);
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8;

%% GIF setup
gifFile = '../../data/maps/IOTFactoryMapwithExp_1/IOTFactoryOccupancyMap.gif';
delayTime = 0.01;
hFig = figure('Position', [100 100 800 800]);

%% Add scans and build GIF
for i = 1:numScans
    [isScanAccepted, ~, ~] = addScan(slamAlg, scans{i});
    if ~isScanAccepted
        fprintf('Scan %d rejected (too close to previous scan)\n', i);
        continue; 
    end
    fprintf('Added scan %d\n', i);
    
    % Build current occupancy map
    [scansNow, posesNow] = scansAndPoses(slamAlg);
    mapNow = buildMap(scansNow, posesNow, mapResolution, maxLidarRange);
    occNow = occupancyMatrix(mapNow);

    % Create PGM-style image
    imgNow = uint8(127 * ones(size(occNow)));
    imgNow(occNow >= mapNow.OccupiedThreshold) = 0;
    imgNow(occNow <= mapNow.FreeThreshold) = 255;
    
    % Display map
    imagesc(imgNow);
    colormap(gray);
    axis equal tight;
    hold on;
    
    % Plot robot trajectory
    if ~isempty(posesNow)
        xPos = posesNow(:,1);
        yPos = posesNow(:,2);
        [rows, ~] = size(imgNow);
        xImg = round((xPos - mapNow.GridLocationInWorld(1)) / (1/mapResolution));
        yImg = round((yPos - mapNow.GridLocationInWorld(2)) / (1/mapResolution));
        yImg = rows - yImg; % invert y to match image coordinates
        plot(xImg, yImg, 'r-', 'LineWidth', 2);
        plot(xImg(end), yImg(end), 'ro', 'MarkerFaceColor','r'); % current robot
    end
    
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
end

fprintf('✅ GIF with robot path saved: %s\n', gifFile);
