function refpath = HybridAStar_local(Map, robotCurrentPose, path, orientation)
    % robotCurrentPose : [x y theta]
    % path : Map에서 4가지 엣지 지점
    % orientation : 각 라인별 바라볼 각도

    % path planning
    ss = stateSpaceSE2;
    ss.StateBounds = [Map.XWorldLimits; Map.YWorldLimits; [-pi pi]];

    sv = validatorOccupancyMap(ss);
    sv.Map = Map;
    sv.ValidationDistance = 0.4;
    
    planner = plannerHybridAStar(sv,'MinTurningRadius',1,'MotionPrimitiveLength',0.5);
    goal = [path(1,1:2) orientation(1)];
    refpath = plan(planner,robotCurrentPose',goal);
    figure; show(planner)
end