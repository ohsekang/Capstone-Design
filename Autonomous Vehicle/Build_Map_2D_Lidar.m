%% Build Map

clear; clc;

load('scans_12(final).mat');
% load('data12f.mat');
scans = scans(1:5:end);

%%
maxRange = 12; % meters
resolution = 20; % cells per meter

slamObj = lidarSLAM(resolution,maxRange);
slamObj.LoopClosureThreshold = 360;
slamObj.LoopClosureSearchRadius = 8;

for i = 1:numel(scans)

    addScan(slamObj,scans{i});
    
    if rem(i,10) == 0
        show(slamObj);
        pause(0.01);
    end
%     show(slamObj);
%     pause(0.01);
end

%%
[scansSLAM,poses] = scansAndPoses(slamObj);
Map = buildMap(scansSLAM,poses,resolution,maxRange);
figure
show(Map)
title('Occupancy Map')