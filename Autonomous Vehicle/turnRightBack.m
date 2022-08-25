function robotCurrentPose = turnRightBack(robotCurrentPose, robot, scout, TxMsgs)
    sampleTime = 0.54; % 0.545;
    vizRate = rateControl(1/sampleTime);
    j=1;
    v = -0.1;
    o = -0.3;
    while j <= 10 % 우측 회전 60도 (예상보다 조금 덜 도는듯)
        
        vv = typecast(swapbytes(int16(v*1000)), "uint8");
        om = typecast(swapbytes(int16(o*1000)), "uint8");
        TxMsgs.Data = ([vv om 0 0 0 0]);
        transmit(scout,TxMsgs)

        % orientation 보정
        vel = derivative(robot, robotCurrentPose, [v o]);

        % Update the current pose
        robotCurrentPose = robotCurrentPose + vel*sampleTime;
        j=j+1;
        waitfor(vizRate);
    end
end