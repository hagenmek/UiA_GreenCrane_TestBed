function Fi = Phi(q,t)

phid = pi/2;   % Rotational angle between local coordinate of body 2 and 3;


% S prime vectors
Sp0A(1:2,1) = [-0.42, 1.055]';
Sp1B(1:2,1) = [-3.139, -0.064]';
Sp2A(1:2,1) = [0, -0.25]';
Sp3C(1:2,1) = [0.4, 0]';
Sp1C(1:2,1) = [-2.589, -0.252]';

%Cartesian coordinates
r1 = q(1:2,1);
phi1 = q(3,1);
r2 = q(4:5,1);
phi2 = q(6,1);
r3 = q(7:8,1);
phi3 = q(9,1);

A1 = A(phi1);
A2 = A(phi2);
A3 = A(phi3);



%Constraint vector Fi(q,t) 
Fi(1:9,1) = 0; 
                                          
Fi(1:2,1) = r1 + A1*Sp1B - Sp0A;          % Revelute joint connection B, body 0-1
Fi(3:4,1) = r3 + A3*Sp3C - r1 - A1*Sp1C;  % Revelute joint connection C, body 1-3
Fi(5:6,1) = r2 + A2*Sp2A;                 % Revelute joint connection A, body 0-2
Fi(7,1) = phi2 - phi3 + phid;             % Translational joint connection, body 2-3  
d1 = r3 - r2;                             % Translational joint connection, body 2-3     
Fi(8,1) = (A2*[1;0])'*d1;                 % Translational joint connection, body 2-3   
d2 = r3 + A3*Sp3C - r2 - A2*Sp2A;         % Longitudinal driver, body 2-3  
Fi(9,1) = d2'*d2 - (CylinderDrive(t))^2;  % Longitudinal driver, body 2-3

