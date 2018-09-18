close all
clear all
clc

x = 100;    % x in [-180,180] (roll)
y = -10;    % y in [-90,90]   (pitch)
z = 0;      % z left unused   (yaw)

i = 1;
for x = -180:10:180
    for y = -90:10:90
        
        results(i,1) = x;
        results(i,2) = y;
        % disp('Ground truth:')
        % disp(sprintf('x = %f', x))
        % disp(sprintf('y = %f', y))

        x = x * pi/180;     
        y = y * pi/180;     
        z = z * pi/180;    
        % syms x y z real;
        
        Rx = [1     0       0
              0  cos(x) sin(x)
              0 -sin(x) cos(x)];
        Ry = [cos(y) 0 -sin(y)
                  0  1      0
              sin(y) 0  cos(y)];
        Rz = [cos(z) sin(z) 0
             -sin(z) cos(z) 0
                  0       0 1];
        bRw = Rx*Ry*Rz;

        gb = bRw * [0;0;1];

        x = atan2( gb(2),gb(3)) * 180/pi;
        y = atan2(-gb(1),norm(gb(2:3))) * 180/pi;

        % disp('Initialization results:')
        % disp(sprintf('Roll = %f', x))
        % disp(sprintf('Pitch = %f', y))
        
        results(i,3) = x;
        results(i,4) = y;
        i = i+1;
    end
end

if(sum(results(:,2)-results(:,4)) + ...
   sum(results(:,2)-results(:,4)) < 1e-5)
    disp('This works!')
end