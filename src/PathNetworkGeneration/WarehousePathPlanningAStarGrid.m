%% Load Occupancy Map
load('../../data/maps/Warehouse/wareHouseOccupancyMap.mat', 'map');

%% Inflate map for safety
inflatedMap = copy(map);
inflate(inflatedMap, 0.3);

%% Display map for interactive selection
figure;
show(map); hold on; axis equal;
title('Click Start (green) and Goal (blue) points on the map');

%% Interactive selection of start and goal
[x, y, ~] = ginput(2);   % user must click start and goal
if numel(x) < 2
    error('You must select both start and goal points.');
end
startXY = [x(1), y(1)];
goalXY  = [x(2), y(2)];

%% Plot start and goal
plot(startXY(1), startXY(2), 'go', 'MarkerFaceColor','g', 'MarkerSize',8);
plot(goalXY(1), goalXY(2), 'bo', 'MarkerFaceColor','b', 'MarkerSize',8);

%% Convert to grid indices
startGrid = world2grid(inflatedMap, startXY);
goalGrid  = world2grid(inflatedMap, goalXY);

%% Planner
planner = plannerAStarGrid(inflatedMap);

%% Plan path
[pathGrid, ~] = plan(planner, startGrid, goalGrid);

if isempty(pathGrid)
    error('No path found between start and goal.');
end

%% Convert path back to world coordinates
pathWorld = grid2world(inflatedMap, pathGrid);

%% Plot result
plot(pathWorld(:,1), pathWorld(:,2), 'r-', 'LineWidth',2);
title('A* Path from Start to Goal');
legend('Start','Goal','Planned Path');
