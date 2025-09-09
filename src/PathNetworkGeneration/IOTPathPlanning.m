%% Load Occupancy Map
mapData = load('localization-map-1599.mat', 'mapPadded');
map = mapData.mapPadded; % extract variable
show(map)
disp(map.GridLocationInWorld);
