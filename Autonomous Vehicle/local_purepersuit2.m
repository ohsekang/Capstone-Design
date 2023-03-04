% 차량의 전역 경로까지 경로 추종

function robotCurrentPose = local_purepersuit2(local_controller, robot, scout, distanceTonNighborPoint, robotCurrentPose, local_path, TxMsgs, pRPLIDAR, Map, path, orientation)
    sampleTime = 0.54; % 0.545;
    vizRate = rateControl(1/sampleTime);
    goalRadius = 0.3; % 0.2

    % 가까운 포인트까지 pure persuit로 이동
    while distanceTonNighborPoint > goalRadius
        tic
        while toc < 0.25 % 0.2에서는 안끊김
            [flag,cnt] = rplidar_flagcnt(pRPLIDAR);
    %         disp(cnt)
            if flag == 1
                break
            end
        end

        if flag == 1 % 객체 검출
        % 검출되는 순간 정지
        TxMsgs.Data = ([0 0 0 0 0 0 0 0]);
        transmit(scout,TxMsgs)
        
        % lidar에서 오차가 발생하여 미리 정해진 각도 만큼 회전
        robotCurrentPose = turnRightBack(robotCurrentPose, robot, scout, TxMsgs);

        % path planning
        refpath = HybridAStar_local(Map, robotCurrentPose, path, orientation); 

        % local path planning result
        local_path = refpath.States(:, 1:2);
        local_controller.Waypoints = local_path;
        distanceTonNighborPoint = norm(robotCurrentPose(1:2,:) - local_path(end,:)');
        robotCurrentPose = local_purepersuit(local_controller, robot, scout, distanceTonNighborPoint, robotCurrentPose, local_path, TxMsgs);
        break;

        else
        % Compute the controller outputs, i.e., the inputs to the robot
        [v, omega] = local_controller(robotCurrentPose);
    
        % Get the robot's velocity using controller inputs
        vel = derivative(robot, robotCurrentPose, [v omega]);
    
        % Update the current pose
        robotCurrentPose = robotCurrentPose + vel*sampleTime;
    
        % Re-compute the distance to the goal
        distanceTonNighborPoint = norm(robotCurrentPose(1:2,:) - local_path(end,:)');
    
        vv = typecast(swapbytes(int16(v*1000)), "uint8");
        ome = typecast(swapbytes(int16(omega*1000)), "uint8");
    
        TxMsgs.Data = ([vv ome 0 0 0 0]);
        transmit(scout,TxMsgs)
        waitfor(vizRate);
        end
    end
end