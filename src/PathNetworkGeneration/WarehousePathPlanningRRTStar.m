%% RRT* planning with robust path shortening (shortenpath + fallback)
clear; close all;

%% Load Occupancy Map
load('../../data/maps/wareHouseOccupancyMap.mat', 'map');

%% Create 2D RRT* State Space & Validator
bounds = [map.XWorldLimits; map.YWorldLimits; [-pi pi]];
ss = stateSpaceSE2(bounds);

stateValidator = validatorOccupancyMap(ss);
stateValidator.Map = map;
stateValidator.ValidationDistance = 0.05;

%% Planner (RRT*)
planner = plannerRRTStar(ss, stateValidator);
planner.MaxConnectionDistance = 3.0;
planner.MaxIterations = 5000;

%% Choose start & goal (interactive). You can also set startSE2/goalSE2 manually.
figure;
show(map);
hold on; axis equal;
title('Click Start (green) and Goal (blue) points on the map (or press Enter to use defaults)');

% Attempt interactive selection; fallback to defaults if user presses Enter
try
    [x, y, button] = ginput(2);   % click start, then goal
    if numel(x) < 2
        error('No clicks detected');
    end
    startSE2 = [x(1), y(1), 0];
    goalSE2  = [x(2), y(2), 0];
catch
    % Default start/goal (edit as needed)
    startSE2 = [-14.0, 1.0, 0];
    goalSE2  = [-2.0, -1.5, 0];
    fprintf('Using default start/goal: [%g %g] -> [%g %g]\n', ...
            startSE2(1), startSE2(2), goalSE2(1), goalSE2(2));
end

% Plot start and goal
plot(startSE2(1), startSE2(2), 'go', 'MarkerFaceColor','g', 'MarkerSize',8);
plot(goalSE2(1), goalSE2(2), 'bo', 'MarkerFaceColor','b', 'MarkerSize',8);

%% Validate Start & Goal
if ~isStateValid(stateValidator, startSE2)
    error('Start state is not valid (inside obstacle or out of bounds).');
end
if ~isStateValid(stateValidator, goalSE2)
    error('Goal state is not valid (inside obstacle or out of bounds).');
end

%% Plan Path
[pthObj, solnInfo] = plan(planner, startSE2, goalSE2);

if ~solnInfo.IsPathFound
    error('No path found between start and goal.');
end

%% Try to shorten using shortenpath (Navigation Toolbox R2024b+). If not available, fallback to randomized shortcutting.
try
    % Preferred: use shortenpath (works with navPath + state validator)
    shortenedPath = shortenpath(pthObj, stateValidator);
    pathStates = shortenedPath.States;   % Nx3 (x,y,theta)
    fprintf('Path shortened using shortenpath().\n');
catch shortenErr
    originalStates = pthObj.States;     % Nx3
    % randomizedShorten returns Nx3 states
    pathStates = randomizedShorten(originalStates, stateValidator, 300);
    fprintf('Path shortened using randomized shortcutting (iterations=%d).\n', 300);
end

%% Plot the final (shortened) path
if size(pathStates,1) < 2
    error('Resulting path has fewer than 2 states.');
end

pathXY = pathStates(:,1:2);

figure;
show(map); hold on; axis equal;
plot(pathXY(:,1), pathXY(:,2), 'r-', 'LineWidth', 2);
plot(startSE2(1), startSE2(2), 'go', 'MarkerFaceColor','g', 'MarkerSize',8);
plot(goalSE2(1), goalSE2(2), 'bo', 'MarkerFaceColor','b', 'MarkerSize',8);
legend('Planned Path','Start','Goal');
title('RRT* Planned Path (shortened)');

%% Local function: randomizedShorten
% Tries random shortcutting between non-consecutive waypoints using isMotionValid.
function shortenedStates = randomizedShorten(states, validator, numIter)
    % states: N x 2 or N x 3
    if size(states,2) == 2
        states = [states, zeros(size(states,1),1)];
    end
    rng('shuffle');
    for it = 1:numIter
        n = size(states,1);
        if n <= 2
            break;
        end
        % pick random i < j with gap > 1
        i = randi([1, n-1]);
        j = randi([i+1, n]);
        if j <= i+1
            continue; % adjacent points, nothing to shortcut
        end
        s1 = states(i,:);
        s2 = states(j,:);
        % Check if direct motion between s1 and s2 is valid
        [isValid, ~] = isMotionValid(validator, s1, s2);
        if isValid
            % remove intermediate nodes between i and j
            states = [states(1:i,:); states(j:end,:)];
        end
    end
    shortenedStates = states;
end
