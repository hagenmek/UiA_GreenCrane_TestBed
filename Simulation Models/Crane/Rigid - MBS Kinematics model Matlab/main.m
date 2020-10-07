clc 
clear all
close all

% MAS703 Project Step2 - Longitudinal Driver AC, 12 DEC 2016 

tmin = 0;             % Start Time
tmax = 15;            % Ending Time
dt = 0.01;            % Step Time
t = [tmin:dt:tmax]';  % Analysis Time
N = size(t,1);        % Tumber of times to perform analysis

% S prime vectors
Sp0A(1:2,1) = [-0.42, 1.055]';
Sp1B(1:2,1) = [-3.139, -0.064]';
Sp2A(1:2,1) = [-0.25, 0]';
Sp3C(1:2,1) = [0.4, 0]';
Sp1C(1:2,1) = [-2.589, -0.252]';

% Initial quess for the cardasian body coordinates 
q = [2.6048, 1.8966, 0.2510, 0.0390, 0.2469, -0.1567, 0.0970, 0.6144, 1.4141]';
fun = @(q)Phi(q, t(N));
q = fsolve(fun, q);

for i=1:N
q = fzerom_par('Phi', q, t(i,1)); %Calling the Newton-Raphson
%q = NewtonR('Phi','jacobian', q, t(i,1));

%qd = jacobian(q)\[0, 0, omega]';
%qdd = jacobian(q)\RHacc(q,qd,t);

qPlot(i,1:9) = q(1:9,1);
%qdPlot(i,1:3) = q(1:3,1);
%qddPlot(i,1:3) = q(1:3,1);
end

figure
plot(t,qPlot);

figure
plot(t,CylinderDrive(t));
grid

%% Animation 
LW1=2;
LW2=1;
LW3=1;


la = 0.20;
lt = 0.1;
va = 30/180*pi;
Coord = CoordSys2D(la,lt,va);

% Geometri Arm
b1 = 0.08;
r_BCx = 3.139;
r_BCy = 0.064;
Larm = 3.68;
Harm = 0.3;

Arm = [[-r_BCx-b1; -r_BCy-Harm/2], [-r_BCx-b1; Harm/2-r_BCy], [Larm-r_BCx; Harm/2-r_BCy], [Larm-r_BCx; -r_BCy-Harm/2], [-r_BCx-b1; -r_BCy-Harm/2]];

% Geometri Cylinder
LWpist = 3.5;
LWcyl =7.5;

Piston = [[-0.2; 0], [0.4; 0]];
Cylinder = [[0; -0.25], [0; 0.472]];


% Geometri Base
LW_base = 10;
b2 = 0.2;
Hbase = 1.94;
Wbase = 0.15; %cm

%Base = [[-Wbase/2; -Hbase + b2], [-Wbase/2; b2], [Wbase/2; b2], [Wbase/2; -Hbase + b2], [-Wbase/2; -Hbase + b2]]; 

BaseV1 = [[-Wbase; b2],[-Wbase; -Hbase + b2]];
BaseV2 = [[Wbase; b2],[Wbase; -Hbase + b2]];
BaseH1 = [[-Wbase; 0],[2.1; 0]];
BaseH2 = [[-Wbase; 0],[0.3; 0]];
BaseAng = [[0.15; -0.1],[1.8; -1.7]];

h = figure(100);
set(100, 'units', 'Centimeter'); % Sætter arbejdsenheden til cm
set(100,'Position',[1 1 22 17]); % Sætter størrelsen af figuren på skærmen [koordinater til nederste venstre hjørne, bredde, højde]

tStart = cputime();

while ishandle(h)
    cla
    
    tElapsed = cputime() - tStart;
    
     [ret,i] = min(abs(t - tElapsed));
    
    r1=qPlot(i,1:2)'; 
    r2=qPlot(i,4:5)';
    r3=qPlot(i,7:8)';
    
    A1=A(qPlot(i,3));
    A2=A(qPlot(i,6));
    A3=A(qPlot(i,9));
    
 % Plot Local coordinate system of Body 1    
    C_1=A1*Coord;
    C_1(1:2,:) = [r1(1,1) + C_1(1,:); r1(2,1) + C_1(2,:)];
    
 % Plot Local coordinate system of Body 2    
    C_2=A2*Coord;
    C_2(1:2,:) = [r2(1,1) + C_2(1,:); r2(2,1) + C_2(2,:)];
    
  % Plot Local coordinate system of Body 3    
    C_3=A3*Coord;
    C_3(1:2,:) = [r3(1,1) + C_3(1,:); r3(2,1) + C_3(2,:)];
    
 % Plot Revelute Joint in connection A   
    A0=A(0);
    r0=[0;0];
    C_A =A0*Coord;
    C_A(1:2,:) = [r0(1,1) + C_A(1,:); r0(2,1) + C_A(2,:)];
    
    %Plot Revelute Joint in connection B  
     C_B(1:2,:)= [Sp0A(1,1) + Coord(1,:) ; Sp0A(2,1) + Coord(2,:)];
     
    %Plot Revelute Joint in connection C
    C_C = A1*Coord; 
    %A3 = A(qPlot(i,12));
    rC = A1*Sp1C(1:2,1);
    C_C(1:2,:)= [(r1(1,1) +  rC(1,1)) + C_C(1,:) ; (r1(2,1) + rC(2,1)) + C_C(2,:)];

    ArmPlot = A1*Arm;
    ArmPlot(1,:)=r1(1,1)+ArmPlot(1,:);
    ArmPlot(2,:)=r1(2,1)+ArmPlot(2,:);
    
    PistonPlot = A3*Piston;
    PistonPlot(1,:)=r3(1,1)+PistonPlot(1,:);
    PistonPlot(2,:)=r3(2,1)+PistonPlot(2,:);
    
    CylinderPlot = A2*Cylinder;
    CylinderPlot(1,:)=r2(1,1)+CylinderPlot(1,:);
    CylinderPlot(2,:)=r2(2,1)+CylinderPlot(2,:);
    
    BasePlotV1 = BaseV1;
    BasePlotV1(1,:) = Sp0A(1,1)+  BasePlotV1(1,:);
    BasePlotV1(2,:) = Sp0A(2,1)+  BasePlotV1(2,:);
    
    BasePlotV2 = BaseV2;
    BasePlotV2(1,:) = Sp0A(1,1)+  BasePlotV2(1,:);
    BasePlotV2(2,:) = Sp0A(2,1)+  BasePlotV2(2,:);
    
    BasePlotH1 = BaseH1;
    BasePlotH1(1,:) = -0.5 + BasePlotH1(1,:);
    BasePlotH1(2,:) = -0.61 + BasePlotH1(2,:);
    
    BasePlotH2 = BaseH2;
    BasePlotH2(1,:) = -0.5 + BasePlotH2(1,:);
    BasePlotH2(2,:) = 1.17 + BasePlotH2(2,:);
    
    BasePlotAng = BaseAng;
    BasePlotAng(1,:) = Sp0A(1,1)+  BasePlotAng(1,:);
    BasePlotAng(2,:) = Sp0A(2,1)+  BasePlotAng(2,:);
    
    
    hold on  
    patch(ArmPlot(1,:),ArmPlot(2,:),'g-','LineWidth',LW1)
    plot(PistonPlot(1,:),PistonPlot(2,:),'b-','LineWidth',LWpist)
    plot(CylinderPlot(1,:),CylinderPlot(2,:),'r-','LineWidth',LWcyl)
    plot(BasePlotV1(1,:),BasePlotV1(2,:),'k-','LineWidth',LW_base)
    plot(BasePlotV2(1,:),BasePlotV2(2,:),'k-','LineWidth',LW_base)
    plot(BasePlotH1(1,:),BasePlotH1(2,:),'k-','LineWidth',LW_base)
    plot(BasePlotH2(1,:),BasePlotH2(2,:),'k-','LineWidth',LW_base)
    plot(BasePlotAng(1,:),BasePlotAng(2,:),'k-','LineWidth',LW_base)
    
    plot(C_1(1,:),C_1(2,:),'m-','LineWidth',LW3)
    plot(C_2(1,:),C_2(2,:),'m-','LineWidth',LW3)
    plot(C_3(1,:),C_3(2,:),'m-','LineWidth',LW3)
    plot(C_A(1,:),C_A(2,:),'m-','LineWidth',LW3)
    plot(C_B(1,:),C_B(2,:),'m-','LineWidth',LW3)
    plot(C_C(1,:),C_C(2,:),'m-','LineWidth',LW3)
    %plot(2*Coord(1,:),2*Coord(2,:),'b-','LineWidth',LW2)
    hold off
    set(gca,'Units','Centimeter','Position',[1,1,20,15])
    axis equal
    box on
    grid on
    axis([-1 4 -1 4])
    title(['t = ',num2str(t(i))])
    
    %Joint Discription
    text(Coord(1,2)-0.25,Coord(2,2)-0.1,'A','FontSize',10)
    text( C_B(1,2)-0.25, C_B(2,2)+0.32,'B','FontSize',10)
    text( C_C(1,2)+0.01, C_C(2,2)-0.1,'C','FontSize',10)


    
    drawnow;
    
    if(i == length(t))
        break;
        pause
    end

end

 