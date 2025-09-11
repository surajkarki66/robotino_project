%% Load Occupancy Map
mapData = load('../../data/maps/localization-map.mat', 'map');
map = mapData.map; % extract variable
show(map)

% Set origin from YAML
map.GridLocationInWorld = [-12.206, -13.810];

% Extract occupancy matrix
occMatrix = occupancyMatrix(map);

% Swap axes: transpose the occupancy matrix
occMatrixSwapped = occMatrix';

% Flip horizontally (left-right)
occMatrixFlippedH = fliplr(occMatrixSwapped);

% Create new occupancy map with horizontally flipped matrix
mapSwapped = occupancyMap(occMatrixFlippedH, map.Resolution);

% Swap origin coordinates accordingly (no addition)
mapSwapped.GridLocationInWorld = map.GridLocationInWorld([2,1]);

%% Plot swapped & horizontally flipped map (for visualization only)
figure;
show(mapSwapped);
title('Occupancy Map of IOT Factory');

%% Print the origin of the final map
disp('GridLocationInWorld of the final map:');
disp(mapSwapped.GridLocationInWorld);

%% Save occupancy matrix as .pgm
% Convert occupancy values to 0-255 grayscale (free=white, occupied=black)
occImage = uint8(255 * (1 - occMatrixFlippedH)); % invert colors
occImage = flipud(occImage);                    % optional: flip vertically for ROS
imwrite(occImage, 'rotated_swapped_flipped_map.pgm');

%% Save the occupancy map as .mat
save('new_map.mat', 'map');

%% Save raw occupancy grid as high-quality PNG (no axes/grid/labels)
imwrite(occImage, 'new_map.png', 'png');

disp('Map saved as .pgm, .mat, and .png successfully (PNG same style as .pgm).');
