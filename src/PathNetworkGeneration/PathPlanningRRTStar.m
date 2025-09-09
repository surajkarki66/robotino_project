%% Load Occupancy Map
load('../../data/maps/IOT.mat','map');
map.GridLocationInWorld = [-39.975, -39.975];

%% Define Docking Nodes
coords_docking = [
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

%% Define Parking Nodes
coords_parking = [
    -15.2618,  0.852601
    -14.215,   0.83
    -0.50355, -0.949287
    -0.276,    0.067
    -0.161829, 1.52521
];

%% Combine nodes for planning
coords_nodes = [coords_docking; coords_parking];
numNodes = size(coords_nodes,1);

%% Keep track of parking indices
parkingIdx = (size(coords_docking,1)+1):numNodes;

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
plot(coords_nodes(:,1), coords_nodes(:,2), 'ko', 'MarkerFaceColor','g');
plot(coords_stations(:,1), coords_stations(:,2), 'yo', 'MarkerFaceColor','y');
title('IoT Factory Map with Nodes and Stations');

%% Use original map for planning (no inflation)
plannerMap = map;

%% Create 2D RRT* state space & validator
bounds = [plannerMap.XWorldLimits; plannerMap.YWorldLimits; [-pi pi]];
ss = stateSpaceSE2(bounds);

stateValidator = validatorOccupancyMap(ss);
stateValidator.Map = plannerMap;
stateValidator.ValidationDistance = 0.05;

%% Planner (RRT*)
planner = plannerRRTStar(ss, stateValidator);
planner.MaxConnectionDistance = 3.0;
planner.MaxIterations = 10000;

%% Build adjacency matrix & store paths
adjMatrix = inf(numNodes);
paths = cell(numNodes);
maxEdgeDist = 4.0;  % only connect nearby nodes

for i = 1:numNodes
    for j = i+1:numNodes
        % Skip connections between parking nodes
        if ismember(i, parkingIdx) && ismember(j, parkingIdx)
            continue;
        end
        
        if norm(coords_nodes(i,:) - coords_nodes(j,:)) <= maxEdgeDist
            startSE2 = [coords_nodes(i,:), 0];
            goalSE2  = [coords_nodes(j,:), 0];

            if isStateValid(stateValidator, startSE2) && isStateValid(stateValidator, goalSE2)
                try
                    [pthObj, solnInfo] = plan(planner, startSE2, goalSE2);
                    if solnInfo.IsPathFound
                        shortenedPath = shortenpath(pthObj, stateValidator);
                        cost = pathLength(shortenedPath);
                        adjMatrix(i,j) = cost;
                        adjMatrix(j,i) = cost;
                        paths{i,j} = shortenedPath.States(:,1:2); % x,y only
                        paths{j,i} = flipud(shortenedPath.States(:,1:2));
                    end
                catch
                    fprintf('No path between node %d and %d\n', i,j);
                end
            end
        end
    end
end

%% Build MATLAB graph
G = graph(adjMatrix);

%% Plot obstacle-free navigation graph with green paths & turning points
figure; show(map); hold on; axis equal;
plot(coords_nodes(:,1), coords_nodes(:,2), 'ko', 'MarkerFaceColor','g');
plot(coords_stations(:,1), coords_stations(:,2), 'yo', 'MarkerFaceColor','y');

turnThreshold = cosd(10); % turning points > 10 degrees

for e = 1:numedges(G)
    [s,t] = findedge(G,e);
    if ~isempty(paths{s,t})
        path = paths{s,t};

        % Plot path in green
        plot(path(:,1), path(:,2), 'g-', 'LineWidth',1.5);

        % Detect significant turning points
        turningPts = [];
        for k = 2:size(path,1)-1
            v1 = path(k,:) - path(k-1,:);
            v2 = path(k+1,:) - path(k,:);
            if norm(v1) > 0 && norm(v2) > 0
                cosTheta = dot(v1,v2)/(norm(v1)*norm(v2));
                if cosTheta < turnThreshold
                    turningPts = [turningPts; path(k,:)];
                end
            end
        end

        % Plot turning points as green circles
        if ~isempty(turningPts)
            plot(turningPts(:,1), turningPts(:,2), 'go', 'MarkerFaceColor','g', 'MarkerSize',6);
        end
    end
end

title('Obstacle-Free Navigation Graph');
legend('Nodes','Stations','Graph edges');
