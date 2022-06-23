vrep=remApi('remoteApi');
vrep.simxFinish(-1); % just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

flag=0;
count=0;
  
    while flag~=1 
        % Get Joints' Handles and Show Connection Status
        [res,joint1]=vrep.simxGetObjectHandle(clientID,'redundantRob_joint1',vrep.simx_opmode_blocking);
        [~,joint2]=vrep.simxGetObjectHandle(clientID,'redundantRob_joint2',vrep.simx_opmode_blocking);
        [~,joint3]=vrep.simxGetObjectHandle(clientID,'redundantRob_joint3',vrep.simx_opmode_blocking);
        [~,joint4]=vrep.simxGetObjectHandle(clientID,'redundantRob_joint4',vrep.simx_opmode_blocking);
        [~,joint5]=vrep.simxGetObjectHandle(clientID,'redundantRob_joint5',vrep.simx_opmode_blocking);
        [~,joint6]=vrep.simxGetObjectHandle(clientID,'redundantRob_joint6',vrep.simx_opmode_blocking);
        [~,joint7]=vrep.simxGetObjectHandle(clientID,'redundantRob_joint7',vrep.simx_opmode_blocking);
        
        if res==vrep.simx_return_ok           
            disp('Connected');
            flag=1;
        else
            disp('Failed to Connect');
            flag=0;
        end
        count=count+1;
        if count>=10
            break
        end
    end
    
	% Get dt (Dynamic Time Step) from V-rep
    [res,dt]=vrep.simxGetFloatingParameter(clientID,vrep.sim_floatparam_simulation_time_step,vrep.simx_opmode_blocking);

    % Simulation duration
    N=5;   %Second
       

    if clientID>-1
        
        vrep.simxSynchronous(clientID, true);
        vrep.simxStartSimulation(clientID, vrep.simx_opmode_oneshot);
        
      	%Streaming operation request
        vrep.simxGetJointPosition(clientID,joint1,vrep.simx_opmode_streaming);
        vrep.simxGetJointPosition(clientID,joint2,vrep.simx_opmode_streaming);
        vrep.simxGetJointPosition(clientID,joint3,vrep.simx_opmode_streaming);
        vrep.simxGetJointPosition(clientID,joint4,vrep.simx_opmode_streaming);
        vrep.simxGetJointPosition(clientID,joint5,vrep.simx_opmode_streaming);
        vrep.simxGetJointPosition(clientID,joint6,vrep.simx_opmode_streaming);
        vrep.simxGetJointPosition(clientID,joint7,vrep.simx_opmode_streaming);

        joint1_pos=zeros(floor(N/dt)+1,1);
        joint2_pos=zeros(floor(N/dt)+1,1);
        joint3_pos=zeros(floor(N/dt)+1,1);
        joint4_pos=zeros(floor(N/dt)+1,1);
        joint5_pos=zeros(floor(N/dt)+1,1);
        joint6_pos=zeros(floor(N/dt)+1,1);
        joint7_pos=zeros(floor(N/dt)+1,1);
        
        i=1;
        while (vrep.simxGetConnectionId(clientID)~=-1)
         vrep.simxSynchronousTrigger(clientID);
         vrep.simxGetPingTime(clientID);
         
            % Simulation time
            simulationTime=vrep.simxGetLastCmdTime(clientID);
            simulationTime=simulationTime*0.001;
	
            % Kp, Ki & Kd factors for all controllers (can be specified
            % separately for each joint)
      
            
            [pid1, joint1_pos(i)] = myPID(clientID,vrep,joint1,90,15,0,0,N,dt,i);            
            [pid2, joint2_pos(i)] = myPID(clientID,vrep,joint2,45,10,0.01,0,N,dt,i);            
            [pid3, joint3_pos(i)] = myPID(clientID,vrep,joint3,5,10,1,0.3,N,dt,i);            
            [pid4, joint4_pos(i)] = myPID(clientID,vrep,joint4,30,12,0,0.4,N,dt,i);            
            [pid5, joint5_pos(i)] = myPID(clientID,vrep,joint5,90,10,0.5,0,N,dt,i);            
            [pid6, joint6_pos(i)] = myPID(clientID,vrep,joint6,30,8,1,0.1,N,dt,i);            
            [pid7, joint7_pos(i)] = myPID(clientID,vrep,joint7,45,12,1.2,0.1,N,dt,i);
                        
            vrep.simxPauseCommunication(clientID, 1);
            
            vrep.simxSetJointTargetVelocity(clientID,joint1,pid1(i),vrep.simx_opmode_oneshot);
            vrep.simxSetJointTargetVelocity(clientID,joint2,pid2(i),vrep.simx_opmode_oneshot);
            vrep.simxSetJointTargetVelocity(clientID,joint3,pid3(i),vrep.simx_opmode_oneshot);
            vrep.simxSetJointTargetVelocity(clientID,joint4,pid4(i),vrep.simx_opmode_oneshot);
            vrep.simxSetJointTargetVelocity(clientID,joint5,pid5(i),vrep.simx_opmode_oneshot);
            vrep.simxSetJointTargetVelocity(clientID,joint6,pid6(i),vrep.simx_opmode_oneshot);
            vrep.simxSetJointTargetVelocity(clientID,joint7,pid7(i),vrep.simx_opmode_oneshot);
            
            vrep.simxPauseCommunication(clientID, 0);            

            % Set a break point
            if simulationTime>=N
                break;
            end 
            i=i+1;
        end
        
        time=linspace(0,N,i-2);
        plot(time,joint1_pos(1:i-2).*180/pi)
        figure
        plot(time,joint2_pos(1:i-2).*180/pi)
        figure
        plot(time,joint3_pos(1:i-2).*180/pi)
        figure
        plot(time,joint4_pos(1:i-2).*180/pi)
        figure
        plot(time,joint5_pos(1:i-2).*180/pi)
        figure
        plot(time,joint6_pos(1:i-2).*180/pi)
        figure
        plot(time,joint7_pos(1:i-2).*180/pi)
        
        vrep.simxStopSimulation(clientID,vrep.simx_opmode_blocking);
        vrep.simxFinish(clientID);
        disp('Program ended')
    end