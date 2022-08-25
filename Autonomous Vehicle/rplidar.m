% % % lidar 연결
% hardwarex_init;
% pRPLIDAR = CreateRPLIDAR();
% [result] = ConnectRPLIDAR(pRPLIDAR, 'RPLIDAR0.txt')
%%
% [result, distance, angle, bNewScan, quality] = GetScanDataResponseRPLIDAR(pRPLIDAR);
% str = sprintf('Distance at %f deg = %f m\n', angle*180.0/pi, distance);
% 
% fig = figure('Position',[200 200 400 400],'NumberTitle','off');
% % Force the figure to have input focus (required to capture keys).
% set(fig,'WindowStyle','Modal'); axis('off');
% scale = 10;
% a = 360*2;
% count = 0; alldistances = zeros([1 100]); allangles = zeros([1 100]);
% i=1;
% while (count <= a)
%     [~, distances, angles, ~, ~] = GetScanDataResponseRPLIDAR(pRPLIDAR);
%     
%     % +-30도에 해당하는 값만 받아옴
%     if -0.5236 <= angles && angles <= 0.5236
%         alldistances(i+1) = distances;
%         allangles(i+1) = angles;
%         i=i+1;
%     end
% 
%     %if bNewScan
%     if count == a
%     %if count > 720/32
% 
%         scan = lidarScan(alldistances, allangles);
% %         clf; hold on; axis([-scale,scale,-scale,scale]);
% %         plot(alldistances.*cos(allangles), alldistances.*sin(allangles), '.');
% %         pause(0.01); key = get(gcf,'CurrentCharacter');
%     end
%     count = count+1;
% end
%%
a = 360*2;
count = 0; 
% alldistances = zeros([1 100]); allangles = zeros([1 100]);
cnt=0;
flag=0;
while (count <= a)
    [~, distances, angles, ~, ~] = GetScanDataResponseRPLIDAR(pRPLIDAR);
    
    % +-10도에 해당하는 값만 받아옴
    n = 10;
    rad = deg2rad(n);
    if ((-rad <= angles && angles <= rad) && (0.2 < distances && distances <= 0.5))
        cnt=cnt+1;
    end

    if cnt >= 40
        flag = 1;
        break
    end
    count = count+1;
end
disp(cnt)
%%
% close(fig);
% 
% [result] = DisconnectRPLIDAR(pRPLIDAR)
% DestroyRPLIDAR(pRPLIDAR);
% clear pRPLIDAR; % unloadlibrary might fail if all the variables that use types from the library are not removed...
% unloadlibrary('hardwarex');