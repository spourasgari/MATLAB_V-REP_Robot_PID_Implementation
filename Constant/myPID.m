function [targetVel, output_pos] = myPID(clientID,vrep,joint,targetPos,Kp,Ki,Kd,N,dt,i)
            
           
            % Feedback Position
            jointposfb = zeros(floor(N/dt)+1,1);  %rad
            
            % Reference Velocity
            targetVel = zeros(floor(N/dt)+1,1);  %Nm

            % Target position
            tarPos = zeros(floor(N/dt)+1,1)+(targetPos*pi/180);  %rad
            
            % Position error
            err = zeros(floor(N/dt)+1,1);     %rad
            Ierr = zeros(floor(N/dt)+1,1);    %rad
            Derr = zeros(floor(N/dt)+1,1);    %rad
            
            [~,jointposfb(i)]=vrep.simxGetJointPosition(clientID,joint,vrep.simx_opmode_buffer);
            
            output_pos = jointposfb(i);
            
            err(i) = tarPos(i)-jointposfb(i);
            
            Ierr(i) = sum(err)*dt;
            
            if i>1
                Derr(i) = (err(i)-err(i-1))/dt;
            else
                Derr(i) = 0;
            end
            
            targetVel(i) = Kp*err(i)+Ki*Ierr(i)+Kd*Derr(i);
            
end