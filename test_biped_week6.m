% =======================================================================
%   OCP2NLP
%   Copyright (c) 2005 by
%   Raktim Bhattacharya, (raktim@aero.tamu.edu)
%   Department of Aerospace Engineering
%   Texas A&M University.
%   All right reserved.
% =======================================================================
clear all;
global nlp;

addpath('C:\Users\Rohan\Documents\Georgia Tech - Undergrad\The End (Fall 2017)\ECE 4560\Optragen-refactor');
addpath('C:\Users\Rohan\Documents\Georgia Tech - Undergrad\The End (Fall 2017)\ECE 4560\Optragen-refactor\src');
% SNOPTPATH = '../../../../snopt';
SNOPTPATH = 'C:\Users\Rohan\Documents\Georgia Tech - Undergrad\The End (Fall 2017)\ECE 4560\Optragen-refactor\snopt';
addpath([ SNOPTPATH ]);
addpath([ SNOPTPATH '/matlab/matlab/' ]);
addpath([ SNOPTPATH '/matlab/mex/' ]);

% Typesetting for figure text
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

ninterv = 2;
hl = 1.0;

% l_1 = 0.5; l_2 = 0.5;   % link lengths
l_1 = 2.00; l_2 = 3.625; l_3 = 3.625;

% initial/final end-effector poses
x0 = -4.5; 
y0 = 0;
theta0 = 0;
xf = 4.5;
yf = 0;
thetaf = 0;
com_back = -.5
com_front = 1.75;
com_height_top =  l_1 + l_2 + l_3;


eps = 0.0001;

% Generate symbolic representation of end-effector pose
syms a1 a2 a3 a4 a5 a6   % a1 = joint 1 angle; a2 = joint 2 angle
% g0_1 = [ cos(a1) -sin(a1) 0 ; ...
%          sin(a1) cos(a1)  -l_1 ; ...
%          0 0 1 ];    % spatial frame to 1st link frame (co-located with first joint)
% g1_2 = [ cos(a2) -sin(a2) 0 ; ...
%          sin(a2) cos(a2)  -l_2 ; ...
%          0 0 1 ];    % 1st link frame to 2nd link frame (co-located with second joint)
% g2_3 = [ cos(a3) -sin(a3) 0 ; ...
%          sin(a3) cos(a3) -l_3 ; ...
%          0 0 1 ];    % 2nd link frame to end frame at end of 2nd link (ie. end-effector)
% g_ee = g0_1*g1_2*g2_3;      % spatial frame to end-effector frame
% g_ee_x_str = char(vpa(g_ee(1, 3), 9));      % end-effector x- spatial position
g_ee = [  cos(a1 + a2 + a3 - a4 - a5 - a6), sin(a1 + a2 + a3 - a4 - a5 - a6), (29*sin(a2 + a3))/8 - (29*sin(a1 + a2 + a3 - a4 - a5))/8 - (29*sin(a1 + a2 + a3 - a4))/8 + (29*sin(a3))/8; ...
    -sin(a1 + a2 + a3 - a4 - a5 - a6), cos(a1 + a2 + a3 - a4 - a5 - a6), (29*cos(a2 + a3))/8 - (29*cos(a1 + a2 + a3 - a4 - a5))/8 - (29*cos(a1 + a2 + a3 - a4))/8 + (29*cos(a3))/8; ...
                                 0,                                0,                                                                                                         1];
pos_com_x = (53*cos(a1 + a2 + a3 - a4 - a5 - a6))/600 - (277*sin(a1 + a2 + a3 - a4 - a5))/400 + (1173*sin(a2 + a3))/400 - (389*sin(a1 + a2 + a3 - a4))/300 + (1061*sin(a3))/300 + (42809^(1/2)*cos(a1 + a2 + a3 - atan(200/53)))/600;
pos_com_y = (1173*cos(a2 + a3))/400 - (277*cos(a1 + a2 + a3 - a4 - a5))/400 - (53*sin(a1 + a2 + a3 - a4 - a5 - a6))/600 - (389*cos(a1 + a2 + a3 - a4))/300 + (1061*cos(a3))/300 + (42809^(1/2)*cos(a1 + a2 + a3 + atan(53/200)))/600;
g_ee_x_str = char(vpa(g_ee(1, 3), 9));      % end-effector x- spatial position
g_ee_y_str = char(vpa(g_ee(2, 3), 9));      % end-effector y- spatial position
pos_com_x_str = char(pos_com_x);
pos_com_y_str = char(pos_com_y);
g_ee_theta_str = char(a1 + a2 + a3 + a4 + a5 + a6);
g_torso_left_leg_str = char(a1 + a2 + a3);
g_torso_right_leg_str = char(a4 + a5 + a6);
g_ee_a1_str = char(a1);
g_ee_a2_str = char(a2);
g_ee_a3_str = char(a3);
g_ee_a4_str = char(a4);
g_ee_a5_str = char(a5);
g_ee_a6_str = char(a6);

% Create trajectory variablesbase
% ===========================
a1 = traj('a1', ninterv,2,3); % Arguments are ninterv, smoothness, order
a2 = traj('a2', ninterv,2,3);
a3 = traj('a3', ninterv,2,3);
a4 = traj('a4', ninterv,2,3); % Arguments are ninterv, smoothness, order
a5 = traj('a5', ninterv,2,3);
a6 = traj('a6', ninterv,2,3);

% Create derivatives of trajectory variables
% ==========================================
a1d = deriv(a1, 'a1');
a2d = deriv(a2, 'a2');
a3d = deriv(a3, 'a3');
a4d = deriv(a4, 'a4');
a5d = deriv(a5, 'a5');
a6d = deriv(a6, 'a6');

ParamList = [];
xVars = {'a1'; 'a2'; 'a3'; 'a4'; 'a5'; 'a6'; 'a1d'; 'a2d'; 'a3d'; 'a4d'; 'a5d'; 'a6d'};

% Define constraints
% ==================
Constr = constraint(x0,g_ee_x_str,x0,'initial', xVars) + ... % x(0)
    constraint(y0,g_ee_y_str,y0,'initial', xVars) + ... % y(0)
    constraint(theta0,g_ee_theta_str,theta0,'initial', xVars) + ... % theta(0)
    constraint(xf,g_ee_x_str,xf,'final', xVars) + ...     % Final position, time is normalised
    constraint(yf,g_ee_y_str,yf,'final', xVars) + ...
    constraint(thetaf,g_ee_theta_str,thetaf,'final', xVars) + ... 
    constraint(x0-.5,g_ee_x_str,xf+.5,'trajectory', xVars) + ... % Try to make sure right leg doesn't go too wild, x and y constaints
    constraint(y0,g_ee_y_str,15,'trajectory', xVars) + ... % don't want right leg  to go into the ground
    constraint(-pi/2,g_ee_a1_str,(3*pi)/4,'trajectory',xVars) + ... %Hip angle constraints -- These are arbitrary -- no real reason
    constraint(-pi,g_ee_a4_str, pi/2,'trajectory',xVars) + ...
    constraint(-3*pi/4,g_ee_a2_str,0,'trajectory',xVars) + ... %Left knee cannot bend outward (ie. unnatural angle)
    constraint(-3*pi/4,g_ee_a5_str,0,'trajectory',xVars) + ... %Right knee cannot bend inward (ie. unnatural angle)
    constraint(0,g_ee_a3_str,pi,'trajectory',xVars) + ... % Foot angle constraints -- these are arbitrary as well
    constraint(0,g_ee_a6_str,pi,'trajectory',xVars) + ...
    constraint(0,g_torso_left_leg_str,0,'initial',xVars) + ... %Torso to Left leg needs to start at 0 angle
    constraint(0,g_torso_left_leg_str,0,'final',xVars) + ... %Torso to Left leg needs to end at 0 angle
    constraint(0,g_torso_left_leg_str,0,'trajectory',xVars) + ... %Left leg needs to remain at 0 angle (on ground)
    constraint(0,g_torso_right_leg_str,0,'initial',xVars) + ... %Torso to Right leg needs to start at 0 angle
    constraint(0,g_torso_right_leg_str,0,'final',xVars) + ... ; %Torso to Right leg needs to end at 0 angle
    constraint(com_back,pos_com_x_str,com_front,'trajectory',xVars) + ... % COM x constraint
    constraint(0,pos_com_y_str,com_height_top,'trajectory',xVars); % COM y constraint


% Define Cost Function
% ====================
Cost = cost('a1d^2+a2d^2+a4d^2+a5d^2','trajectory'); % Minimise energy

% Collocation Points, using Gaussian Quadrature formula
% =====================================================

breaks = linspace(0,hl,ninterv+1);
gauss = [-1 1]*sqrt(1/3)/2;
temp = ((breaks(2:ninterv+1)+breaks(1:ninterv))/2);
temp = temp(ones(1,length(gauss)),:) + gauss'*diff(breaks);
colpnts = temp(:).';

HL = [0 colpnts hl];
HL = linspace(0,hl,20);


% Path where the problem related files will be stored
% ===================================================
pathName = './';  % Save it all in the current directory.

% Name of the problem, will be used to identify files
% ===================================================
probName = 'planar2r';

% List of trajectories used in the problem
% ========================================
TrajList = traj.trajList(a1,a1d,a2,a2d,a3,a3d,a4,a4d,a5,a5d,a6,a6d);

nlp = ocp2nlp(TrajList, Cost,Constr, HL, ParamList,pathName,probName);
snset('Minimize');



xlow = -Inf*ones(nlp.nIC,1);
xupp = Inf*ones(nlp.nIC,1);

Time = linspace(0,5,50);
a1_val = linspace(-pi,pi,50);
a2_val = linspace(-pi,pi,50);
a3_val = linspace(-pi,pi,50);
a4_val = linspace(-pi,pi,50);
a5_val = linspace(-pi,pi,50);
a6_val = linspace(-pi,pi,50);
a1_sp = createGuess(a1,Time,a1_val);
a2_sp = createGuess(a2,Time,a2_val);
a3_sp = createGuess(a3,Time,a3_val);
a4_sp = createGuess(a4,Time,a4_val);
a5_sp = createGuess(a5,Time,a5_val);
a6_sp = createGuess(a6,Time,a6_val);
init = [a1_sp.coefs a2_sp.coefs a3_sp.coefs a4_sp.coefs a5_sp.coefs a6_sp.coefs]';% + 0.001*rand(nlp.nIC,1);
%init = zeros(nlp.nIC,1);

ghSnopt = snoptFunction(nlp);
tic;
[x,F,inform] = snopt(init, xlow, xupp, [], [], ...
                     [0;nlp.LinCon.lb;nlp.nlb], [Inf;nlp.LinCon.ub;nlp.nub],...
                     [], [], ghSnopt);
toc;
F(1)

sp = getTrajSplines(nlp,x);
a1SP = sp{1};
a2SP = sp{2};
a3SP = sp{3};
a4SP = sp{4};
a5SP = sp{5};
a6SP = sp{6};

refinedTimeGrid = linspace(min(HL),max(HL),50);

A1 = fnval(a1SP,refinedTimeGrid);
A1d = fnval(fnder(a1SP),refinedTimeGrid);

A2 = fnval(a2SP,refinedTimeGrid);
A2d = fnval(fnder(a2SP),refinedTimeGrid);

A3 = fnval(a3SP,refinedTimeGrid);
A3d = fnval(fnder(a3SP),refinedTimeGrid);

A4 = fnval(a4SP,refinedTimeGrid);
A4d = fnval(fnder(a4SP),refinedTimeGrid);

A5 = fnval(a5SP,refinedTimeGrid);
A5d = fnval(fnder(a5SP),refinedTimeGrid);

A6 = fnval(a6SP,refinedTimeGrid);
A6d = fnval(fnder(a6SP),refinedTimeGrid);

% Planar 2-R Arm joint trajectory
figure(1);
plot3(A1,A2,A3,'b');
xlabel('Joint 1 (rad)'); ylabel('Joint 2 (rad)'); zlabel('Joint 3 (rad)');
title('Joint Trajectory');
figure(2);
plot3(A4,A5,A6,'b');
xlabel('Joint 4 (rad)'); ylabel('Joint 5 (rad)'); zlabel('Joint 6 (rad)');
title('Joint Trajectory');

a_time = Time;
traj_alpha = [A1; A2; A3; A4; A5; A6];


% =======================================================================
%  Now for our code!
% =======================================================================
% Instantiate instance/object of the Biped() class
my_biped = Biped();

% Week 5 Biped Script 
my_link_lens = [2.00, 3.625, 3.625, 0.5, 1.75; 2.00, 3.625, 3.625, 0.5, 1.75];
my_biped.set_geometry( my_link_lens );
a_alpha = [0 0 0; 0 0 0];
my_biped.set_alpha(a_alpha);
my_biped.set_stance('LEFT_FOOT')

%Angle plots
figure(3)
hold on
subplot(3,1,1)
plot(a_time, traj_alpha(1,:));
title('Angles vs Time for Left Leg');
xlabel('Time (s)');
ylabel('Alpha hip');
subplot(3,1,2)
plot(a_time, traj_alpha(2,:));
xlabel('Time (s)');
ylabel('Alpha knee');
subplot(3,1,3)
plot(a_time, traj_alpha(3,:));
xlabel('Time (s)');
ylabel('Alpha ankle');
hold off

figure(4)
hold on
subplot(3,1,1)
plot(a_time, traj_alpha(4,:));
title('Angles vs Time for Right Leg');
xlabel('Time (s)');
ylabel('Alpha hip');
subplot(3,1,2)
plot(a_time, traj_alpha(5,:));
xlabel('Time (s)');
ylabel('Alpha knee');
subplot(3,1,3)
plot(a_time, traj_alpha(6,:));
xlabel('Time (s)');
ylabel('Alpha ankle');
hold off

%Animation (Need longer time step)
a_time_anim = a_time(1:2:end);
traj_alpha_anim = traj_alpha(:,2:2:end);
my_biped.animateTrajectory(a_time_anim, traj_alpha_anim) 