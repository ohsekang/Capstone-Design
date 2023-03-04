% 차량의 지역 경로까지 경로 추종

function robotCurrentPose = local_purepersuit(local_controller, robot, scout, distanceTonNighborPoint, robotCurrentPose, local_path, TxMsgs)
    sampleTime = 0.54; % 0.545;
    vizRate = rateControl(1/sampleTime);
    goalRadius = 0.3; % 0.2

    % 가까운 포인트까지 pure persuit로 이동
    while distanceTonNighborPoint > goalRadius
    
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