%% data load
% 사전에 12층을 한바퀴 회전하도록 path planning 및 path following을 미리 계산 후 변수로 저장
% lookahead distance : 1, sample time : 0.545로 설정
load lhd1sam0.545.mat

%% plot Map & path 
% occupancy map에서 4가지 way points를 설정함
path = [7 7 -7 -7; 0 12 12 0]';

show(Map)
hold on
plot(path(:,1), path(:,2))
hold off

%% CAN 통신
% scout 객체로 can 연결 후 온라인 모드 설정과 can data를 읽을 수 있도록 모드 변경
clear scout
scout = canChannel('PEAK-System', 'PCAN_USBBUS1');

start(scout)

message = receive(scout, Inf, "OutputFormat", "timetable")
% Table 3.5 Control Mode Setting Frame
TxMsg = canMessage(1057, false, 1);
TxMsg.Data = [1];
transmit(scout,TxMsg)

%% Connect rplidar
% rplidar와 연결하고 연결이 잘 되었는지 result로 확인(0이 나온 경우가 연결된 경우)
hardwarex_init;
pRPLIDAR = CreateRPLIDAR();

[result] = ConnectRPLIDAR(pRPLIDAR, 'RPLIDAR0.txt')

%%
% Define Waypoints
% 초기 위치의 x,y ori를 0으로 설정 (실제 시작 위치는 12층 엘리베이터 앞)
robotInitialLocation = [0 0];

initialOrientation = 0;

% 차량의 현재 위치 및 방향
robotCurrentPose = [robotInitialLocation initialOrientation]';

% 차량 모델 설정
robot = differentialDriveKinematics("TrackWidth", 0.4, ...
    "VehicleInputs", "VehicleSpeedHeadingRate", "WheelRadius", 0.085);

% local pure persuit 파라미터 설정
local_controller = controllerPurePursuit;
local_controller.Waypoints = [robotInitialLocation; path(1,:)];
local_controller.DesiredLinearVelocity = 0.2;
local_controller.MaxAngularVelocity = 0.3; % 0.5
local_controller.LookaheadDistance = 0.5; % 1


% refpath = HybridAStar(Map, robotCurrentPose, path, orientation, i);
% local_path = refpath.States(:, 1:2);
% local_controller.Waypoints = local_path;

% Using the Path Following Controller, Drive the Robot over the Desired Waypoints
% 목표 반경 설정
goalRadius = 0.5; % 0.2

% 차량이 움직이면서 끊기지 않는 최대 시간을 sample time으로 설정
% -> 라이다 탐지, 서버 데이터 읽기, 다음 주행의 속도, 각속도 전송까지 시간 확보를 위해 cycle time과 다르게 적용
% Initialize the simulation loop
sampleTime = 0.545; % 0.545;
vizRate = rateControl(1/sampleTime);

 % moving can message
TxMsgs = canMessage(273, false, 8);

% check point index
i=1;

% check point orient
% 4가지 waypoints의 orient를 미리 저장
orientation = [pi/2 -pi -pi/2 0];
flag = 0;
while 1 % 실내에서 계속 돌도록 구현

    % Get lidar object flag
    % 라이다로 0.25초 동안 장애물이 있는지 확인
%     tic
%     while toc < 0.25 % 0.2에서는 안끊김
%         [flag,cnt] = rplidar_flagcnt(pRPLIDAR);
% %         disp(cnt)
%         if flag == 1
%             break
%         end
%     end

    % 전방 +-10도에 장애물 발견하면 정지 -> 후진 + 회전 -> 다음 waypoint까지 새로운 지역 경로를 탐색
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

    % 객체가 없으면 기존에 찾은 전역 경로로 이동
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
    
    % 차량이 waypoint와 가까워지면 다음 waypoint를 가리키도록 변경하고 새로운 경로를 탐색
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
