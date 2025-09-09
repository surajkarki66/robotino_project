%% Load Occupancy Map
load('../../data/maps/IOT.mat','map');
map.GridLocationInWorld = [-39.975, -39.975];

%% Define Docking Points and Parking Points
coords_docking_pts = [
    -13.1368,  2.67684
    -10.7464,  2.74653
    -8.33197,  2.72984
    -7.64,     1.329
    -6.83323, -1.7505
    -5.616,   -0.951
    -5.5545,  -3.00751
    -4.33607, -1.82372
    -2.59474,  0.314477
    -1.747,   -1.418
];

coords_parking_pts = [
    -15.2618,  0.852601
    -14.215,   0.83
    -0.50355, -0.949287
    -0.276,    0.067
    -0.161829, 1.52521
];

coords_nodes = [coords_docking_pts; coords_parking_pts];
numNodes = size(coords_nodes,1);

%% Define Stations
coords_stations = [
-15.39,   2.552
-15.752,  5.251
-5.786,  -1.672
-5.251,  -1.789
-5.456,  -2.146
-5.954,  -1.916
-1.282,  -1.716
-3.875,   1.624
-5.987,   0.631
-7.398,   4.286
-9.824,   4.267
-12.313,  4.249
];

%% Plot map with nodes and stations
figure; show(map); hold on; axis equal;
plot(coords_nodes(:,1), coords_nodes(:,2), 'ko','MarkerFaceColor','g');
plot(coords_stations(:,1), coords_stations(:,2), 'yo','MarkerFaceColor','y');
title('IoT Factory Map with Nodes and Stations');

%% Inflate Map for Planning
inflatedMap = copy(map);
inflate(inflatedMap, 0.3);  % 0.3 meters safety clearance

%% Create A* Planner
planner = plannerAStarGrid(inflatedMap);

%% Build adjacency matrix for nearby obstacle-free paths
adjMatrix = inf(numNodes);
paths = cell(numNodes);

maxEdgeDist = 4.0;  % only connect nearby nodes

for i = 1:numNodes
    for j = i+1:numNodes
        % Skip connections between parking nodes
        if ismember(i, (length(coords_docking_pts)+1):numNodes) && ...
           ismember(j, (length(coords_docking_pts)+1):numNodes)
            continue;
        end
        
        if norm(coords_nodes(i,:) - coords_nodes(j,:)) <= maxEdgeDist
            startGrid = world2grid(inflatedMap, coords_nodes(i,:));
            goalGrid  = world2grid(inflatedMap, coords_nodes(j,:));
            [pathGrid, ~] = plan(planner, startGrid, goalGrid);

            if ~isempty(pathGrid)  % path exists
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

%% Plot paths 
figure; show(map); hold on; axis equal;
plot(coords_nodes(:,1), coords_nodes(:,2), 'ko','MarkerFaceColor','g');
plot(coords_stations(:,1), coords_stations(:,2), 'yo','MarkerFaceColor','y');

for e = 1:numedges(G)
    [s,t] = findedge(G,e);
    if ~isempty(paths{s,t})
        path = paths{s,t};
        plot(path(:,1), path(:,2), 'g-', 'LineWidth',1.5);
    end
end

title('Obstacle-Free Navigation Graph');
legend('Nodes','Stations','Graph edges');
