function robotCurrentPose = turnRight(robotCurrentPose, robot, scout, TxMsgs)
    sampleTime = 0.54; % 0.545;
    vizRate = rateControl(1/sampleTime);
    j=1;
    while j <= 6 % 우측 회전 60도 (예상보다 조금 덜 도는듯)
        om = typecast(swapbytes(int16(-0.3*1000)), "uint8");
        TxMsgs.Data = ([0 0 om 0 0 0 0]);
        transmit(scout,TxMsgs)

        % orientation 보정
        vel = derivative(robot, robotCurrentPose, [0 -0.2]);

        % Update the current pose
        robotCurrentPose = robotCurrentPose + vel*sampleTime;
        j=j+1;
        waitfor(vizRate);
    end
end