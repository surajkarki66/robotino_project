%% Load Occupancy Map
% Load the map from iot.mat and set its world location
load('../../data/maps/iot.mat', 'map');
map.GridLocationInWorld = [-39.975, -39.975];

%% Define Docking Nodes
% Coordinates for docking nodes (in meters)
coords_docking = [
    -13.1368,  2.67684;
    -10.7464,  2.74653;
    -8.33197,  2.72984;
    -7.64,     1.329;
    -6.83323, -1.7505;
    -5.616,   -0.951;
    -5.5545,  -3.00751;
    -4.33607, -1.82372;
    -2.59474,  0.314477;
    -1.747,   -1.418
];

%% Define Parking Nodes
% Coordinates for parking nodes (in meters)
coords_parking = [
    -15.2618,  0.852601;
    -14.215,   0.83;
    -0.50355, -0.949287;
    -0.276,    0.067;
    -0.161829, 1.52521
];

%% Combine Nodes for Planning
% Combine docking and parking nodes
coords_nodes = [coords_docking; coords_parking];
numNodes = size(coords_nodes, 1);

%% Keep Track of Parking Indices
% Indices for parking nodes (11 to 15)
parkingIdx = (size(coords_docking, 1) + 1):numNodes;

%% Define Stations
% Coordinates for stations (in meters)
coords_stations = [
    -15.39,   2.552;
    -15.752,  5.251;
    -5.786,  -1.672;
    -5.251,  -1.789;
    -5.456,  -2.146;
    -5.954,  -1.916;
    -1.282,  -1.716;
    -3.875,   1.624;
    -5.987,   0.631;
    -7.398,   4.286;
    -9.824,   4.267;
    -12.313,  4.249
];

%% Plot Map with Nodes and Stations
figure;
show(map);
hold on;
axis equal;

% Plot docking nodes in green
plot(coords_docking(:, 1), coords_docking(:, 2), 'ko', 'MarkerFaceColor', 'g');

% Plot parking nodes in blue
plot(coords_parking(:, 1), coords_parking(:, 2), 'ko', 'MarkerFaceColor', 'b');

% Plot stations in yellow
plot(coords_stations(:, 1), coords_stations(:, 2), 'yo', 'MarkerFaceColor', 'y');

title('IoT Factory Map with Nodes and Stations');
legend('Docking Nodes', 'Parking Nodes', 'Stations');

%% Use Original Map for Planning
plannerMap = map;

%% Create 2D RRT* State Space & Validator
bounds = [plannerMap.XWorldLimits; plannerMap.YWorldLimits; [-pi pi]];
ss = stateSpaceSE2(bounds);

stateValidator = validatorOccupancyMap(ss);
stateValidator.Map = plannerMap;
stateValidator.ValidationDistance = 0.05;

%% Planner (RRT*)
planner = plannerRRTStar(ss, stateValidator);
planner.MaxConnectionDistance = 3.0;
planner.MaxIterations = 10000;

%% Build Adjacency Matrix & Store Paths
adjMatrix = inf(numNodes);
paths = cell(numNodes);
maxEdgeDist = 4.0;

for i = 1:numNodes
    for j = i + 1:numNodes
        if ismember(i, parkingIdx) && ismember(j, parkingIdx)
            continue;
        end
        if norm(coords_nodes(i, :) - coords_nodes(j, :)) <= maxEdgeDist
            startSE2 = [coords_nodes(i, :), 0];
            goalSE2 = [coords_nodes(j, :), 0];
            if isStateValid(stateValidator, startSE2) && isStateValid(stateValidator, goalSE2)
                try
                    [pthObj, solnInfo] = plan(planner, startSE2, goalSE2);
                    if solnInfo.IsPathFound
                        shortenedPath = shortenpath(pthObj, stateValidator);
                        cost = pathLength(shortenedPath);
                        adjMatrix(i, j) = cost;
                        adjMatrix(j, i) = cost;
                        paths{i, j} = shortenedPath.States(:, 1:2);
                        paths{j, i} = flipud(shortenedPath.States(:, 1:2));
                    end
                catch
                    fprintf('No path between node %d and %d\n', i, j);
                end
            end
        end
    end
end

%% Build MATLAB Graph
G = graph(adjMatrix);

%% Plot Obstacle-Free Navigation Graph
figure;
show(map);
hold on;
axis equal;

% Plot nodes and stations
plot(coords_docking(:, 1), coords_docking(:, 2), 'ko', 'MarkerFaceColor', 'g');
plot(coords_parking(:, 1), coords_parking(:, 2), 'ko', 'MarkerFaceColor', 'b');
plot(coords_stations(:, 1), coords_stations(:, 2), 'yo', 'MarkerFaceColor', 'y');

turnThreshold = cosd(10);

for e = 1:numedges(G)
    [s, t] = findedge(G, e);
    if ~isempty(paths{s, t})
        path = paths{s, t};

        % Plot path in green
        plot(path(:, 1), path(:, 2), 'g-', 'LineWidth', 1.5);

        % Detect significant turning points
        turningPts = [];
        for k = 2:size(path, 1) - 1
            v1 = path(k, :) - path(k - 1, :);
            v2 = path(k + 1, :) - path(k, :);
            if norm(v1) > 0 && norm(v2) > 0
                cosTheta = dot(v1, v2) / (norm(v1) * norm(v2));
                if cosTheta < turnThreshold
                    turningPts = [turningPts; path(k, :)];
                end
            end
        end
    end
end

title('Obstacle-Free Navigation Graph');
legend('Docking Nodes', 'Parking Nodes', 'Stations', 'Graph edges');

%% Generate navigation-paths.xml
% Write paths with nodes and turning points to XML file
fid = fopen('../../data/maps/navigation-paths.xml', 'w');
fprintf(fid, '<root>\n');
fprintf(fid, '<paths>\n');
path_id = 1;
for e = 1:numedges(G)
    [s, t] = findedge(G, e);
    if ~isempty(paths{s, t})
        path = paths{s, t};
        
        % Detect significant turning points
        turningPts = [];
        for k = 2:size(path, 1) - 1
            v1 = path(k, :) - path(k - 1, :);
            v2 = path(k + 1, :) - path(k, :);
            if norm(v1) > 0 && norm(v2) > 0
                cosTheta = dot(v1, v2) / (norm(v1) * norm(v2));
                if cosTheta < turnThreshold
                    turningPts = [turningPts; path(k, :)];
                end
            end
        end
        
        % Write forward path (direction=1)
        fprintf(fid, '<path width="0.85" id="%d" direction="1">\n', path_id);
        fprintf(fid, '<nodes>\n');
        node_id = 1;
        fprintf(fid, '<node id="%d" posid="%d">\n', node_id, s);
        fprintf(fid, '<x>%.6f</x>\n', path(1, 1));
        fprintf(fid, '<y>%.6f</y>\n', path(1, 2));
        fprintf(fid, '</node>\n');
        for k = 1:size(turningPts, 1)
            node_id = node_id + 1;
            fprintf(fid, '<node id="%d">\n', node_id);
            fprintf(fid, '<x>%.6f</x>\n', turningPts(k, 1));
            fprintf(fid, '<y>%.6f</y>\n', turningPts(k, 2));
            fprintf(fid, '</node>\n');
        end
        node_id = node_id + 1;
        fprintf(fid, '<node id="%d" posid="%d">\n', node_id, t);
        fprintf(fid, '<x>%.6f</x>\n', path(end, 1));
        fprintf(fid, '<y>%.6f</y>\n', path(end, 2));
        fprintf(fid, '</node>\n');
        fprintf(fid, '</nodes>\n');
        fprintf(fid, '</path>\n');
        path_id = path_id + 1;
        
        % Write reverse path (direction=2)
        fprintf(fid, '<path width="0.85" id="%d" direction="2">\n', path_id);
        fprintf(fid, '<nodes>\n');
        node_id = 1;
        fprintf(fid, '<node id="%d" posid="%d">\n', node_id, t);
        fprintf(fid, '<x>%.6f</x>\n', path(end, 1));
        fprintf(fid, '<y>%.6f</y>\n', path(end, 2));
        fprintf(fid, '</node>\n');
        for k = size(turningPts, 1):-1:1
            node_id = node_id + 1;
            fprintf(fid, '<node id="%d">\n', node_id);
            fprintf(fid, '<x>%.6f</x>\n', turningPts(k, 1));
            fprintf(fid, '<y>%.6f</y>\n', turningPts(k, 2));
            fprintf(fid, '</node>\n');
        end
        node_id = node_id + 1;
        fprintf(fid, '<node id="%d" posid="%d">\n', node_id, s);
        fprintf(fid, '<x>%.6f</x>\n', path(1, 1));
        fprintf(fid, '<y>%.6f</y>\n', path(1, 2));
        fprintf(fid, '</node>\n');
        fprintf(fid, '</nodes>\n');
        fprintf(fid, '</path>\n');
        path_id = path_id + 1;
    end
end
fprintf(fid, '</paths>\n');
fprintf(fid, '<areas/>\n');
fprintf(fid, '</root>\n');
fclose(fid);

%% Generate positions.lisp
% Write node coordinates (in millimeters) to LISP file
fid = fopen('../../data/maps/positions.lisp', 'w');
for i = 1:numNodes
    x = coords_nodes(i, 1) * 1000;
    y = coords_nodes(i, 2) * 1000;
    fprintf(fid, '((IS-A LOCATION) (NAME %d) (TYPE POSE) (APPROACH-TYPE (PATH-NAV))\n', i);
    fprintf(fid, ' (APPROACH-REGION-POSE (%.3f %.3f 0)) (APPROACH-REGION-DIST 75)\n', x, y);
    fprintf(fid, ' (APPROACH-EXACT-POSE (%.3f %.3f 0)) (APPROACH-EXACT-DIST 100)\n', x, y);
    fprintf(fid, ' (APPROACH-EXACT-SAFETYCL 0) (ORIENTATION-REGION (ANGLE-ABSOLUTE 0))\n');
    fprintf(fid, ' (ORIENTATION-EXACT (ANGLE-ABSOLUTE 0)) (PARKING-STATE FREE))\n');
end
fclose(fid);

%% Generate stations.lisp
% Write station coordinates and closest node indices to LISP file
fid = fopen('../../data/maps/stations.lisp', 'w');
station_types = {'CP-F-AASS', 'CP-F-AASS', 'CP-PS-AP6', 'CP-PS-AP5', ...
                 'CP-PS-AP2_4', 'CP-PS-AP1', 'CP-F-BOXES', 'CP-F-ASRS20', ...
                 'CP-F-DOCK', 'CP-F-RASS', 'CP-F-RASS', 'CP-F-RASS'};
for i = 1:size(coords_stations, 1)
    % Find closest node
    dists = sqrt(sum((coords_nodes - coords_stations(i, :)).^2, 2));
    [~, closest_node] = min(dists);
    fprintf(fid, '((IS-A STATION) (ID %d) (TYPE %s) (APPROACH-LOCATION %d)\n', ...
            i, station_types{i}, closest_node);
    fprintf(fid, ' (POSE (%.3f %.3f 0.0 0.0d0 0.0 0.0)))\n', ...
            coords_stations(i, 1), coords_stations(i, 2));
end
fclose(fid);