function jac = jacobian(q)

% S prime vectors
Sp0(1:2,1) = [-0.42, 1.055]';
Sp1(1:2,1) = [-3.139, -0.064]';
Sp2(1:2,1) = [-0.25, 0]';
Sp3(1:2,1) = [0.4, 0]';


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
B1 = B(phi1);
B2 = B(phi2);
B3 = B(phi3);

I=eye(2);

% Matrix Initial
jac(1:3, 1:3) = 0; 

%jacobian for revelute joing connected to ground
jac(1:2, 1:3) = [-I, -B1*Sp1(1:2,1)];
jac(3,3) = 1; 