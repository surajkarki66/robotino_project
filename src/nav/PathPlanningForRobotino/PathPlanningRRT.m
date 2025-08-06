% Plan Mobile Robot Paths Using RRT

% Load Occupancy Map
load('IOTFactoryOccupancyGridMap.mat', 'map');
figure;
show(map);
hold on;

% Set start and goal poses
start = [39.975, 39.975, pi];
goal = [30.031, 43.035, pi/2];

% Show start and goal positions of robot
plot(start(1), start(2), 'ro');
plot(goal(1), goal(2), 'mo');

% Show start and goal heading angle using a line
r = 0.5;
plot([start(1), start(1) + r*cos(start(3))], [start(2), start(2) + r*sin(start(3))], 'r-');
plot([goal(1), goal(1) + r*cos(goal(3))], [goal(2), goal(2) + r*sin(goal(3))], 'm-');
hold off;

% Inflate the map for path planning
inflatedMap = copy(map);
inflate(inflatedMap, 0.1);

% Define State Space
bounds = [inflatedMap.XWorldLimits; inflatedMap.YWorldLimits; [-pi pi]];
ss = stateSpaceDubins(bounds);
ss.MinTurningRadius = 0.4;

% Create Validator
stateValidator = validatorOccupancyMap(ss);
stateValidator.Map = inflatedMap;
stateValidator.ValidationDistance = 0.05;

% Create Path Planner
planner = plannerRRT(ss, stateValidator);
planner.MaxConnectionDistance = 2.5;
planner.MaxIterations = 30000;
planner.GoalReachedFcn = @exampleHelperCheckIfGoal;

% Plan the Path
rng default;
[pthObj, solnInfo] = plan(planner, start, goal);

% Shorten Path
shortenedPath = shortenpath(pthObj, stateValidator);

% Compute Path Lengths
originalLength = pathLength(pthObj);
shortenedLength = pathLength(shortenedPath);

% Plot Original and Shortened Path
figure;
show(map);
hold on;

% Plot search tree
plot(solnInfo.TreeData(:,1), solnInfo.TreeData(:,2), '.-');

% Interpolate and plot original path
interpolate(pthObj, 300);
plot(pthObj.States(:,1), pthObj.States(:,2), 'b-', 'LineWidth', 2);

% Interpolate and plot shortened path
interpolate(shortenedPath, 300);
plot(shortenedPath.States(:,1), shortenedPath.States(:,2), 'g-', 'LineWidth', 3);

% Show start and goal
plot(start(1), start(2), 'ro');
plot(goal(1), goal(2), 'mo');
legend('Search Tree', 'Original Path', 'Shortened Path');
hold off;
