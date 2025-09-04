%% Load Occupancy Map
load('IOTFactoryOccupancyGridMap.mat', 'map');

% Set origin for the map (world frame offset)
map.GridLocationInWorld = [-39.975, -39.975];

%% Define Nodes
coords_nodes = [
-15.2618,  0.852601 %12
-14.215,   0.83     %11
-13.1368,  2.67684  %1
-10.7464,  2.74653  %2
-8.33197,  2.72984  %3
-7.64,     1.329    %4
-6.83323, -1.7505   %7
-5.616,   -0.951    %10
-5.5545,  -3.00751  %8
-4.33607, -1.82372  %9
-2.59474,  0.314477 %5
-1.747,   -1.418    %6
-0.50355, -0.949287 %15
-0.276,    0.067    %13
-0.161829, 1.52521  %14
];

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

%% First Figure: Map with nodes & stations (world frame)
figure;
show(map); hold on; axis equal;
plot(coords_nodes(:,1),    coords_nodes(:,2),    'go', 'MarkerSize', 6, 'LineWidth', 1.5, 'MarkerFaceColor', 'g');
plot(coords_stations(:,1), coords_stations(:,2), 'yo', 'MarkerSize', 6, 'LineWidth', 1.5, 'MarkerFaceColor', 'y');
legend('Nodes (Green)', 'Stations (Yellow)');

%% Start and Goal in WORLD coordinates (x,y)
startWorld = [-2.47543, -3.5765];
goalWorld  = [-13.1368,   2.67684];

% Plot start/goal
plot(startWorld(1), startWorld(2), 'ro', 'MarkerFaceColor', 'r');
plot(goalWorld(1),  goalWorld(2),  'mo', 'MarkerFaceColor', 'm');

%% Inflate the map for planning
inflatedMap = copy(map);
inflate(inflatedMap, 0.1);

%% Planner (A* Grid)
planner = plannerAStarGrid(inflatedMap);

%% Convert world -> grid (expects integer grid indices)
startGrid = world2grid(inflatedMap, startWorld);  % [row col]
goalGrid  = world2grid(inflatedMap, goalWorld);   % [row col]

%% Plan path in GRID frame
rng default;
[pathGrid, solnInfo] = plan(planner, startGrid, goalGrid);  % pathGrid is N×2 [row col]

% Optional: check success
if isempty(pathGrid)
    error('A* could not find a path. Check start/goal are in free space and inflation radius.');
end

%% Convert GRID path -> WORLD path for plotting
pathWorld = grid2world(inflatedMap, pathGrid);   % N×2 [x y]

%% Compute path length in world units
pathLengthVal = sum(vecnorm(diff(pathWorld,1,1), 2, 2));
fprintf('Path Length = %.3f (world units)\n', pathLengthVal);

%% Second Figure: Map with nodes, stations, and A* path
figure;
show(map); hold on; axis equal;
plot(coords_nodes(:,1),    coords_nodes(:,2),    'go', 'MarkerSize', 6, 'LineWidth', 1.5, 'MarkerFaceColor', 'g');
plot(coords_stations(:,1), coords_stations(:,2), 'yo', 'MarkerSize', 6, 'LineWidth', 1.5, 'MarkerFaceColor', 'y');

% Plot path (world frame)
plot(pathWorld(:,1), pathWorld(:,2), 'g-', 'LineWidth', 3);

% Start/goal markers
plot(startWorld(1), startWorld(2), 'ro', 'MarkerFaceColor', 'r');
plot(goalWorld(1),  goalWorld(2),  'mo', 'MarkerFaceColor', 'm');

legend('Nodes (Green)', 'Stations (Yellow)', 'A* Path', 'Start', 'Goal');
