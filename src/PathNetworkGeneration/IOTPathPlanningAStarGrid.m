%% Load Occupancy Map
load('../../data/maps/iotFactoryOccupancyMap.mat', 'map');
map.GridLocationInWorld = [-39.975, -39.975];

%% Define Docking Points and Parking Points
coords_docking_pts = [
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

coords_parking_pts = [
    -15.2618,  0.852601;
    -14.215,   0.83;
    -0.50355, -0.949287;
    -0.276,    0.067;
    -0.161829, 1.52521
];

coords_nodes = [coords_docking_pts; coords_parking_pts];
numNodes = size(coords_nodes, 1);

%% Define Stations
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

% Docking nodes in green
plot(coords_docking_pts(:,1), coords_docking_pts(:,2), 'ko', 'MarkerFaceColor', 'g');

% Parking nodes in blue
plot(coords_parking_pts(:,1), coords_parking_pts(:,2), 'ko', 'MarkerFaceColor', 'b');

% Stations in yellow
plot(coords_stations(:,1), coords_stations(:,2), 'yo', 'MarkerFaceColor', 'y');

title('IoT Factory Map with Nodes and Stations');
legend('Docking Nodes', 'Parking Nodes', 'Stations');

%% Inflate Map for Planning
inflatedMap = copy(map);
inflate(inflatedMap, 0.3);

%% Create A* Planner
planner = plannerAStarGrid(inflatedMap);

%% Build Adjacency Matrix & Paths
adjMatrix = inf(numNodes);
paths = cell(numNodes);
maxEdgeDist = 4.0;

for i = 1:numNodes
    for j = i+1:numNodes
        if ismember(i, (length(coords_docking_pts)+1):numNodes) && ...
           ismember(j, (length(coords_docking_pts)+1):numNodes)
            continue;
        end
        if norm(coords_nodes(i,:) - coords_nodes(j,:)) <= maxEdgeDist
            startGrid = world2grid(inflatedMap, coords_nodes(i,:));
            goalGrid  = world2grid(inflatedMap, coords_nodes(j,:));
            [pathGrid, ~] = plan(planner, startGrid, goalGrid);

            if ~isempty(pathGrid)
                pathWorld = grid2world(inflatedMap, pathGrid);
                cost = sum(vecnorm(diff(pathWorld,1,1),2,2));
                adjMatrix(i,j) = cost;
                adjMatrix(j,i) = cost;
                paths{i,j} = pathWorld;
                paths{j,i} = flipud(pathWorld);
            end
        end
    end
end

%% Build MATLAB Graph
G = graph(adjMatrix);

%% Plot Paths
figure;
show(map);
hold on;
axis equal;

plot(coords_docking_pts(:,1), coords_docking_pts(:,2), 'ko', 'MarkerFaceColor', 'g');
plot(coords_parking_pts(:,1), coords_parking_pts(:,2), 'ko', 'MarkerFaceColor', 'b');
plot(coords_stations(:,1), coords_stations(:,2), 'yo', 'MarkerFaceColor', 'y');

for e = 1:numedges(G)
    [s,t] = findedge(G, e);
    if ~isempty(paths{s,t})
        path = paths{s,t};
        plot(path(:,1), path(:,2), 'g-', 'LineWidth', 1.5);
    end
end

title('Obstacle-Free Navigation Graph');
legend('Docking Nodes', 'Parking Nodes', 'Stations', 'Graph edges');

%% Generate navigation-paths.xml
% Write paths to XML file, including start and end nodes
fid = fopen('../../data/path_networks/AStar/navigation-paths.xml', 'w');
fprintf(fid, '<root>\n');
fprintf(fid, '<paths>\n');
path_id = 1;
for e = 1:numedges(G)
    [s, t] = findedge(G, e);
    if ~isempty(paths{s, t})
        path = paths{s, t};
        
        % Write forward path (direction=1)
        fprintf(fid, '<path width="0.85" id="%d" direction="1">\n', path_id);
        fprintf(fid, '<nodes>\n');
        for k = 1:size(path, 1)
            fprintf(fid, '<node id="%d"', k);
            if k == 1
                fprintf(fid, ' posid="%d"', s);
            elseif k == size(path, 1)
                fprintf(fid, ' posid="%d"', t);
            end
            fprintf(fid, '>\n');
            fprintf(fid, '<x>%.6f</x>\n', path(k, 1));
            fprintf(fid, '<y>%.6f</y>\n', path(k, 2));
            fprintf(fid, '</node>\n');
        end
        fprintf(fid, '</nodes>\n');
        fprintf(fid, '</path>\n');
        path_id = path_id + 1;
        
        % Write reverse path (direction=2)
        fprintf(fid, '<path width="0.85" id="%d" direction="2">\n', path_id);
        fprintf(fid, '<nodes>\n');
        for k = size(path, 1):-1:1
            fprintf(fid, '<node id="%d"', size(path, 1) - k + 1);
            if k == size(path, 1)
                fprintf(fid, ' posid="%d"', t);
            elseif k == 1
                fprintf(fid, ' posid="%d"', s);
            end
            fprintf(fid, '>\n');
            fprintf(fid, '<x>%.6f</x>\n', path(k, 1));
            fprintf(fid, '<y>%.6f</y>\n', path(k, 2));
            fprintf(fid, '</node>\n');
        end
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
fid = fopen('../../data/path_networks/AStar/positions.lisp', 'w');
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
fid = fopen('../../data/path_networks/AStar/stations.lisp', 'w');
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