%% data load
load lhd1sam0.545.mat
%% plot Map & path 
path = [7 7 -7 -7; 0 12 12 0]';

show(Map)
hold on
plot(path(:,1), path(:,2))
hold off
%% CAN 통신
clear scout
scout = canChannel('PEAK-System', 'PCAN_USBBUS1');

start(scout)

message = receive(scout, Inf, "OutputFormat", "timetable")
% Table 3.5 Control Mode Setting Frame
TxMsg = canMessage(1057, false, 1);
TxMsg.Data = [1];
transmit(scout,TxMsg)
%% Connect rplidar
hardwarex_init;
pRPLIDAR = CreateRPLIDAR();

[result] = ConnectRPLIDAR(pRPLIDAR, 'RPLIDAR0.txt')
%%
% Define Waypoints
robotInitialLocation = [0 0];

initialOrientation = 0;

robotCurrentPose = [robotInitialLocation initialOrientation]';

robot = differentialDriveKinematics("TrackWidth", 0.4, ...
    "VehicleInputs", "VehicleSpeedHeadingRate", "WheelRadius", 0.085);

% local pure persuit
local_controller = controllerPurePursuit;
local_controller.Waypoints = [robotInitialLocation; path(1,:)];
local_controller.DesiredLinearVelocity = 0.2;
local_controller.MaxAngularVelocity = 0.3; % 0.5
local_controller.LookaheadDistance = 0.5; % 1


% refpath = HybridAStar(Map, robotCurrentPose, path, orientation, i);
% local_path = refpath.States(:, 1:2);
% local_controller.Waypoints = local_path;

% Using the Path Following Controller, Drive the Robot over the Desired Waypoints
goalRadius = 0.2;

% Initialize the simulation loop
sampleTime = 0.54; % 0.545;
vizRate = rateControl(1/sampleTime);

 % moving can message
TxMsgs = canMessage(273, false, 8);

% check point index
i=1;

% check point orient
orientation = [pi/2 -pi -pi/2 0];

while 1 % 실내에서 계속 돌도록 구현

    % Get lidar object flag
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
        refpath = HybridAStar(Map, robotCurrentPose, path, orientation, i); 

        % local path planning result
        local_path = refpath.States(:, 1:2);
        local_controller.Waypoints = local_path;
        distanceTonNighborPoint = norm(robotCurrentPose(1:2,:) - local_path(end,:)');
        
        % local pure persuit and orient tunning
        robotCurrentPose = local_purepersuit(local_controller, robot, scout, distanceTonNighborPoint, robotCurrentPose, local_path, TxMsgs);        

    else % 객체 미검출 = pure persuit 그대로 주행

        % Compute the controller outputs, i.e., the inputs to the robot
        [v, omega] = local_controller(robotCurrentPose);

        % Get the robot's velocity using controller inputs
        vel = derivative(robot, robotCurrentPose, [v omega]);

        % Update the current pose
        robotCurrentPose = robotCurrentPose + vel*sampleTime;

        vv = typecast(swapbytes(int16(v*1000)), "uint8");
        ome = typecast(swapbytes(int16(omega*1000)), "uint8");

        TxMsgs.Data = ([vv ome 0 0 0 0]);
        transmit(scout,TxMsgs)
    end

    % edge Point에 근접하면 다음 Point를 가리키도록 i 값 증가
    distanceToPoint = norm(robotCurrentPose(1:2) - path(i,:)');
    if distanceToPoint < goalRadius
        i=rem(i,4)+1;
        refpath = HybridAStar(Map, robotCurrentPose, path, orientation, i);
        local_path = refpath.States(:, 1:2);
        local_controller.Waypoints = local_path;
    end
%     toc
    waitfor(vizRate);
end