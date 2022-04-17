function [dist] = simDist(njoints, nindividuals, setAngles, simTime)
    % Starting simulation bridge
    [sim, clientID] = startSim();
    
%     % defining joint and individuals
%     njoints = 10;
%     nindividuals = 10;
%     simTime = 15;
    
    % joint and goal handles initializations
    jointHandles = zeros(nindividuals, njoints);
    goalHandles = zeros(nindividuals,1);
    
    % initializing poisitons vector
    currPosition = zeros(nindividuals, 3);
    goalPosition = zeros(nindividuals, 3);
    
    % acquiring handles
    % acquiring the joint handles of servo motors
    for i=1:nindividuals
        for j=1:njoints
            jointName = sprintf('/snake0[%d]/Servo_joint_%d', i-1, j-1);
            [~, jointHandles(i,j)] = sim.simxGetObjectHandle(clientID,jointName,sim.simx_opmode_oneshot_wait);
        end
    end

    % acquiring the joint handles of goal positions
    for i=1:nindividuals
            jointName = sprintf('/Dummy[%d]', i-1);
            [~, goalHandles(i)] = sim.simxGetObjectHandle(clientID,jointName,sim.simx_opmode_oneshot_wait);
    end
    
    
    % controlling joint angles
    t=clock;
    startTime=t(6);
    currentTime=t(6);
    while (currentTime-startTime < simTime)
        for i=1:individuals
            for j=1:joints
                retsetjoint = sim.simxSetJointTargetPosition(clientID, jointHandles(i,j), setAngles(i,j), sim.simx_opmode_blocking);
                fprintf("joint angle set successfully");
            end
        end
        t=clock;
        currentTime=t(6);
    end
%     retsetjoint = sim.simxSetJointTargetPosition(clientID, jointHandles(1,5), deg2rad(45), sim.simx_opmode_blocking);
%     if (retsetjoint == sim.simx_return_ok)
%         fprintf("joint angle set successfully");
%     end
    
    % getting positions
    % getting current positions    
    for i=1:nindividuals
        [~, currPosition(i,:)] = sim.simxGetObjectPosition(clientID, jointHandles(i,1), -1, sim.simx_opmode_oneshot_wait);
    end

    % setting goal positions    
    for i=1:nindividuals
        [~, goalPosition(i,:)] = sim.simxGetObjectPosition(clientID, goalHandles(i), -1, sim.simx_opmode_oneshot_wait);
    end

    % getting distances
    for i=1:nindividuals
        dist(i) = currPosition(i,2) - goalPosition(i,2);
    end
end


function [sim, clientID] = startSim()
    sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
    sim.simxFinish(-1); % just in case, close all opened connections
    clientID=sim.simxStart('127.0.0.1',19997,true,true,5000,5);
    if (clientID>-1)
        disp('Connected to remote API server');
        % Now send some data to CoppeliaSim in a non-blocking fashion:
        sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot);
    else
        disp('Failed connecting to remote API server');
        stopSim(sim, clientID)
    end
end

function stopSim(sim, clientID)
    sim.simxPauseSimulation(clientID,sim.simx_opmode_oneshot_wait); % pause simulation
    %sim.simxStopSimulation(clientID,sim.simx_opmode_oneshot_wait); % stop simulation
    sim.simxFinish(clientID);  % close the line if still open
    sim.delete();              % call the destructor!
    disp('simulation ended');
end