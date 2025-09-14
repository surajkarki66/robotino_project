%% Load Lidar and Odometry Data
load("../../data/lidar_data/IndoorLidarScanAndOdometryData.mat")

%% Initialize Lidar Scan Map
MaxLidarRange = 8;
MapResolution = 20;
scanMapObj = lidarscanmap(MapResolution, MaxLidarRange);

% Add first scan
addScan(scanMapObj, messages{1}.scan);

scans = cell(length(messages),1);
scans{1} = messages{1}.scan;
firstTimeLCDetected = false;

%% Initialize Factor Graph for Graph-Based SLAM
fg = factorGraph;
opts = factorGraphSolverOptions;

numExcludeViews = 30;
lcMatchThreshold = 185;
lcSearchRadius = 8;

%% Loop through all scans
for i = 2:length(messages)
    % Try to add scan to lidar scan map
    isScanAccepted = addScan(scanMapObj, messages{i}.scan);
    scans{i} = messages{i}.scan;

    if isScanAccepted
        % Set IDs
        prevPoseID = scanMapObj.NumScans-1;
        currPoseID = scanMapObj.NumScans;

        % Get relative 2-D measurement
        relPoseMeasure = scanMapObj.ConnectionAttributes.RelativePose(end,:);

        % Add relative pose factor
        poseFactor = factorTwoPoseSE2([prevPoseID currPoseID], Measurement=relPoseMeasure);
        addFactor(fg, poseFactor);

        % Initialize current node
        nodeState(fg, currPoseID, messages{i}.Pose);

        % Loop closure detection
        if i > numExcludeViews
            [lcPose, lcPoseID] = detectLoopClosure(scanMapObj, ...
                                                  MatchThreshold=lcMatchThreshold, ...
                                                  SearchRadius=lcSearchRadius, ...
                                                  NumExcludeViews=numExcludeViews);
            if ~isempty(lcPose)
                % Add loop closure factor
                lcFactor = factorTwoPoseSE2([lcPoseID currPoseID], Measurement=lcPose);
                addFactor(fg, lcFactor);

                % Fix first node and optimize
                fixNode(fg,1); 
                optimize(fg, opts);

                % Get optimized poses
                poseIDs = nodeIDs(fg, NodeType="POSE_SE2");
                optimizedPoseStates = nodeState(fg, poseIDs);

                % Visualize first loop closure
                if ~firstTimeLCDetected
                    HelperPlotLidarScanMap(lcPoseID, scanMapObj, optimizedPoseStates);
                    firstTimeLCDetected = true;
                end

                % Update scan positions
                updateScanPoses(scanMapObj, optimizedPoseStates);
            end
        end
    end
end

%% Show final lidar scan map
figure
show(scanMapObj);
title("Final Built Map of Environment with Robot Trajectory")

%% Build Occupancy Grid Map
map = buildMap(scans, scanMapObj.ScanAttributes.AbsolutePose, MapResolution, MaxLidarRange);

figure
show(map);
hold on
show(scanMapObj, ShowMap=false);
hold off
title("Occupancy Grid Map Built Using Lidar SLAM")

%% Save Occupancy Map
save('../../data/maps/indoorAreaOccupancyMap.mat', 'map');

%% Generate Image from Occupancy Map
occMatrix = occupancyMatrix(map);
occupied_thresh = map.OccupiedThreshold;
free_thresh = map.FreeThreshold;

img = uint8(127 * ones(size(occMatrix)));  % Unknown = gray
img(occMatrix >= occupied_thresh) = 0;     % Occupied = black
img(occMatrix <= free_thresh) = 255;       % Free = white

% Save images
imwrite(img, '../../data/maps/indoorAreaOccupancyMap.pgm');
imwrite(img, '../../data/maps/indoorAreaOccupancyMap.png');

% Save YAML file
origin = map.GridLocationInWorld;          
resolution = 1 / map.Resolution;           

fid = fopen('../../data/maps/indoorAreaOccupancyMap.yaml', 'w');
fprintf(fid, 'image: indoorAreaOccupancyMap.pgm\n');
fprintf(fid, 'resolution: %.4f\n', resolution);
fprintf(fid, 'origin: [%.4f, %.4f, 0.0]\n', origin(1), origin(2));
fprintf(fid, 'negate: 0\n');
fprintf(fid, 'occupied_thresh: %.2f\n', occupied_thresh);
fprintf(fid, 'free_thresh: %.2f\n', free_thresh);
fclose(fid);

%% Plot Occupancy Map with Robot Trajectory
imgFlipped = flipud(img);

figure;
imagesc(map.XWorldLimits, map.YWorldLimits, imgFlipped);
axis xy;  % y-axis upward
axis equal tight;
colormap(gray);
hold on;

% Extract robot trajectory from optimized poses
robotTrajectory = optimizedPoseStates(:,1:2);

% Plot trajectory
plot(robotTrajectory(:,1), robotTrajectory(:,2), 'r-', 'LineWidth', 2);       
plot(robotTrajectory(end,1), robotTrajectory(end,2), 'ro', 'MarkerFaceColor','r'); 

title('Indoor Area Occupancy Map with Robot Trajectory');
hold off;

% Save PNG with trajectory overlay
frame = getframe(gcf);
imWithTrajectory = frame2im(frame);
imwrite(imWithTrajectory, '../../data/maps/indoorAreaOccupancyMapwithTrajectory.png');
