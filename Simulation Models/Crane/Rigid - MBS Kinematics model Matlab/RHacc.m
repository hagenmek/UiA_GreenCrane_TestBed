function F = RHacc(q,qd,t)

% S prime vectors
Sp_AB(1:2,1) = [-0.42, 1.055]';
Sp_CB(1:2,1) = [-3.139, -0.064]';

r_A = [0, 0]';
phi_A = 0;
omega = 4;

%Cartesian coordinates
r_AC = q(1:2,1);
phi_CB = q(3,1);

r_ACd = qd(1:2,1);
phi_CBd = qd(3,1);

F(1:3,1) = 0;

F(1:2,1) = phi_CBd^2*A(phi_CB)*Sp_CB(1:2,1);
F(3,1) = 0;