function a = CylinderDrive(t)

phaseA = 0; % Phase angle

A = 0.25; % Aplitude -> (lmax-lmin)/2;
lmean = 1.022; % center position of the piston stroke of the cylinder -> (lmax+lmin)/2
T = 10; % Periodic time



a = A*sin(2*pi/T*t + phaseA) + lmean;