% Load Occupancy Map
load('IOTFactoryOccupancyGridMap.mat', 'map');

% Set origin for the map
map.GridLocationInWorld = [-39.975, -39.975];

% Define Nodes
coords_nodes = [
-15.2618,  0.852601 %12
-14.215,   0.83 %11
-13.1368,  2.67684 %1
-10.7464,  2.74653 %2
-8.33197,  2.72984 %3
-7.64,     1.329 %4
-6.83323, -1.7505 %7
-5.616,   -0.951 %10
-5.5545,  -3.00751 %8
-4.33607, -1.82372 %9
-2.59474,  0.314477 %5
-1.747,   -1.418 %6
-0.50355, -0.949287 %15
-0.276,    0.067 %13
-0.161829, 1.52521 %14
];

% Define Stations 
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

% --- First Figure: Map with nodes & stations ---
figure;
show(map);
hold on;
plot(coords_nodes(:,1), coords_nodes(:,2), 'go', ...
    'MarkerSize', 6, 'LineWidth', 1.5, 'MarkerFaceColor', 'g');
plot(coords_stations(:,1), coords_stations(:,2), 'yo', ...
    'MarkerSize', 6, 'LineWidth', 1.5, 'MarkerFaceColor', 'y');
legend('Nodes (Green)', 'Stations (Yellow)');

% --- Start and Goal ---
start = [-2.47543, -3.5765, pi];
goal = [-13.1368, 2.67684, pi/2];

% Show start and goal positions of robot
plot(start(1), start(2), 'ro', 'MarkerFaceColor', 'r');
plot(goal(1), goal(2), 'mo', 'MarkerFaceColor', 'm');

% Show heading lines
r = 0.5;
plot([start(1), start(1) + r*cos(start(3))], ...
     [start(2), start(2) + r*sin(start(3))], 'r-');
plot([goal(1), goal(1) + r*cos(goal(3))], ...
     [goal(2), goal(2) + r*sin(goal(3))], 'm-');

% --- Inflate the map for planning ---
inflatedMap = copy(map);
inflate(inflatedMap, 0.1);

% --- State Space ---
bounds = [inflatedMap.XWorldLimits; inflatedMap.YWorldLimits; [-pi pi]];
ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.4;

% --- Validator ---
stateValidator = validatorOccupancyMap(ss);
stateValidator.Map = inflatedMap;
stateValidator.ValidationDistance = 0.05;

% --- Planner ---
planner = plannerRRTStar(ss, stateValidator);
planner.MaxConnectionDistance = 2.5;
planner.MaxIterations = 30000;
planner.GoalReachedFcn = @exampleHelperCheckIfGoal;

% --- Plan Path ---
rng default;
[pthObj, solnInfo] = plan(planner, start, goal);

% --- Shorten Path ---
shortenedPath = shortenpath(pthObj, stateValidator);

% --- Path Lengths ---
originalLength = pathLength(pthObj);
shortenedLength = pathLength(shortenedPath);

% --- Second Figure: Path with nodes & stations ---
figure;
show(map);
hold on;
plot(coords_nodes(:,1), coords_nodes(:,2), 'go', ...
    'MarkerSize', 6, 'LineWidth', 1.5, 'MarkerFaceColor', 'g');
plot(coords_stations(:,1), coords_stations(:,2), 'yo', ...
    'MarkerSize', 6, 'LineWidth', 1.5, 'MarkerFaceColor', 'y');

% Plot shortened path
plot(shortenedPath.States(:,1), shortenedPath.States(:,2), ...
    'g-', 'LineWidth', 3);

% Show start and goal
plot(start(1), start(2), 'ro', 'MarkerFaceColor', 'r');
plot(goal(1), goal(2), 'mo', 'MarkerFaceColor', 'm');

legend('Nodes (Green)', 'Stations (Yellow)', 'Shortened Path', 'Start', 'Goal');
