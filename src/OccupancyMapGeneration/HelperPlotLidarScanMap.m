function exampleHelperPlotLidarScanMap(matchScanId, scanMapObj, optimized_poses_fg)
%EXAMPLEHELPERPLOTLIDARSCANMAP Visualize lidar scan map 
%   
%   This function is for internal use only. It may be removed in the
%   future.

%   Copyright 2022 The MathWorks, Inc.

Map = figure(Position=[0 0 900 450]);
tiledlayout(1,2)
axOldMap = nexttile;
show(scanMapObj,Parent=axOldMap);
title('Lidar Scan Map Before Optimization');
% Draw the loop closure edge before optimization
FromPoint = scanMapObj.ScanAttributes.AbsolutePose(scanMapObj.NumScans,1:2);
ToPoint = scanMapObj.ScanAttributes.AbsolutePose(matchScanId,1:2);
hold on
% Draw the loop closure edge after optimization
plot([FromPoint(1) ToPoint(1)], [FromPoint(2) ToPoint(2)],'g',LineWidth=1);
% Last scan text
text(FromPoint(1),FromPoint(2)-1.45,0,['Scan ' num2str(scanMapObj.NumScans)],HorizontalAlignment="center");
text(FromPoint(1),FromPoint(2)-0.45,0,'\uparrow',HorizontalAlignment="center")
% First scan text
text(ToPoint(1),ToPoint(2)+1.65,['Scan ' num2str(matchScanId)],HorizontalAlignment="center");
text(ToPoint(1),ToPoint(2)+0.65,'\downarrow',HorizontalAlignment="center")
updateScanPoses(scanMapObj,optimized_poses_fg);
axUpdatedMap = nexttile;
show(scanMapObj,Parent=axUpdatedMap);
%%
title('Lidar Scan Map After Optimization');
FromPoint = scanMapObj.ScanAttributes.AbsolutePose(scanMapObj.NumScans,1:2);
ToPoint = scanMapObj.ScanAttributes.AbsolutePose(matchScanId,1:2);
hold on

plot([FromPoint(1) ToPoint(1)], [FromPoint(2) ToPoint(2)],'g',LineWidth=1);
% Last scan text
text(FromPoint(1),FromPoint(2)-1.45,0,['Scan ' num2str(scanMapObj.NumScans)],HorizontalAlignment="center");
text(FromPoint(1),FromPoint(2)-0.45,0,'\uparrow',HorizontalAlignment="center")
% First scan text
text(ToPoint(1),ToPoint(2)+1.65,['Scan ' num2str(matchScanId)],HorizontalAlignment="center");
text(ToPoint(1),ToPoint(2)+0.65,'\downarrow',HorizontalAlignment="center")
axis([axOldMap axUpdatedMap],[-2 22 -4 10])
sgtitle("First Loop Closure")
hold off

end