function flag = rplidar_flag(pRPLIDAR)

    a = 360*2;
    cnt=0;
    flag=0;
    for k=1:a
        [~, distances, angles, ~, ~] = GetScanDataResponseRPLIDAR(pRPLIDAR);
    
        % +-10도에 해당하는 값만 받아옴
        n = 10;
        rad = deg2rad(n);
        if ((-rad <= angles && angles <= rad) && (0.2 <= distances && distances <= 0.5))
            cnt=cnt+1;
        end

        if cnt >= 40
            flag = 1;
            break
        end
    end
end