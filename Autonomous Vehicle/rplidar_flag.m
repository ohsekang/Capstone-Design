% 라이다를 통해 scan data를 받아와서 전방 +-10도와 0.2~0.5m 범위 부채꼴에 포함되는 scan data의 수를 저장
% -> 임의로 40개 이상이면 객체가 있다고 판단하여 새로운 경로를 탐색하기 위해 flag 신호를 보냄
% -> 벽과 가깝게 주행하는 것도 감안해서 거리와 각도를 측정

% 사전에 test_rplidar.m 파일에서 ridar 객체를 미리 설정
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