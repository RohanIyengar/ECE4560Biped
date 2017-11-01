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

% addpath('C:\Users\Rohan\Documents\Georgia Tech - Undergrad\The End (Fall 2017)\ECE 4560\Optragen-refactor');
% addpath('C:\Users\Rohan\Documents\Georgia Tech - Undergrad\The End (Fall 2017)\ECE 4560\Optragen-refactor\src');
% % SNOPTPATH = '../../../../snopt';
% SNOPTPATH = 'C:\Users\Rohan\Documents\Georgia Tech - Undergrad\The End (Fall 2017)\ECE 4560\Optragen-refactor\snopt';
% addpath([ SNOPTPATH ]);
% addpath([ SNOPTPATH '/matlab/matlab/' ]);
% addpath([ SNOPTPATH '/matlab/mex/' ]);

% Typesetting for figure text
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultTextInterpreter','latex');

ninterv = 2;
hl = 1.0;

% l_1 = 0.5; l_2 = 0.5;   % link lengths
l_1 = 2.00; l_2 = 3.625; l_3 = 3.625;

% initial/final end-effector poses
x0 = 0; 
y0 = -(l_1 + l_2 + l_3); 
theta0 = 0;
xf = 4;
yf = -1;
thetaf = 0;

eps = 0.0001;

% Generate symbolic representation of end-effector pose
syms a1 a2 a3      % a1 = joint 1 angle; a2 = joint 2 angle
g0_1 = [ cos(a1) -sin(a1) 0 ; ...
         sin(a1) cos(a1)  -l_1 ; ...
         0 0 1 ];    % spatial frame to 1st link frame (co-located with first joint)
g1_2 = [ cos(a2) -sin(a2) 0 ; ...
         sin(a2) cos(a2)  -l_2 ; ...
         0 0 1 ];    % 1st link frame to 2nd link frame (co-located with second joint)
g2_3 = [ cos(a3) -sin(a3) 0 ; ...
         sin(a3) cos(a3) -l_3 ; ...
         0 0 1 ];    % 2nd link frame to end frame at end of 2nd link (ie. end-effector)
g_ee = g0_1*g1_2*g2_3;      % spatial frame to end-effector frame
g_ee_x_str = char(vpa(g_ee(1, 3), 9));      % end-effector x- spatial position
g_ee_y_str = char(vpa(g_ee(2, 3), 9));      % end-effector y- spatial position
g_ee_theta_str = char(a1 + a2 + a3);
g_ee_a1_str = char(a1);
g_ee_a2_str = char(a2);
g_ee_a3_str = char(a3);

% Create trajectory variablesbase
% ===========================
a1 = traj('a1', ninterv,2,3); % Arguments are ninterv, smoothness, order
a2 = traj('a2', ninterv,2,3);
a3 = traj('a3', ninterv,2,3);

% Create derivatives of trajectory variables
% ==========================================
a1d = deriv(a1, 'a1');
a2d = deriv(a2, 'a2');
a3d = deriv(a3, 'a3');

ParamList = [];
xVars = {'a1'; 'a2'; 'a3'; 'a1d'; 'a2d'; 'a3d'};

% Define constraints
% ==================
Constr = constraint(x0,g_ee_x_str,x0,'initial', xVars) + ... % x(0)
    constraint(y0,g_ee_y_str,y0,'initial', xVars) + ... % y(0)
    constraint(theta0,g_ee_theta_str,theta0,'initial', xVars) + ... % theta(0)
    constraint(xf,g_ee_x_str,xf,'final', xVars) + ...     % Final position, time is normalised
    constraint(yf,g_ee_y_str,yf,'final', xVars) + ...
    constraint(thetaf,g_ee_theta_str,thetaf,'final', xVars) + ...
    constraint(-pi/4,g_ee_a1_str,3*pi/4,'trajectory',xVars) + ...
    constraint(-pi/4,g_ee_a2_str,3*pi/4,'trajectory',xVars) + ...
    constraint(-pi,g_ee_a3_str,pi,'trajectory',xVars);

% Define Cost Function
% ====================
Cost = cost('a1d^2+a2d^2','trajectory'); % Minimise energy

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
TrajList = traj.trajList(a1,a1d,a2,a2d,a3,a3d);

nlp = ocp2nlp(TrajList, Cost,Constr, HL, ParamList,pathName,probName);
snset('Minimize');



xlow = -Inf*ones(nlp.nIC,1);
xupp = Inf*ones(nlp.nIC,1);

Time = linspace(0,5,50);
a1_val = linspace(-pi,pi,50);
a2_val = linspace(-pi,pi,50);
a3_val = linspace(-pi,pi,50);
a1_sp = createGuess(a1,Time,a1_val);
a2_sp = createGuess(a2,Time,a2_val);
a3_sp = createGuess(a3,Time,a3_val);
init = [a1_sp.coefs a2_sp.coefs a3_sp.coefs]';% + 0.001*rand(nlp.nIC,1);
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


refinedTimeGrid = linspace(min(HL),max(HL),50);

A1 = fnval(a1SP,refinedTimeGrid);
A1d = fnval(fnder(a1SP),refinedTimeGrid);

A2 = fnval(a2SP,refinedTimeGrid);
A2d = fnval(fnder(a2SP),refinedTimeGrid);

A3 = fnval(a3SP,refinedTimeGrid);
A3d = fnval(fnder(a3SP),refinedTimeGrid);

% Planar 2-R Arm joint trajectory
figure(1);
plot3(A1,A2,A3,'b');
xlabel('Joint 1 (rad)'); ylabel('Joint 2 (rad)'); zlabel('Joint 3 (rad)');
title('Joint Trajectory');

a_time = Time;
traj_alpha = [A1; A2; A3; zeros(3,length(a_time))];


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

%Angle plots
figure
hold on
subplot(3,1,1)
plot(a_time, traj_alpha(1,:));
title('Angles vs Time');
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

%Animation (Need longer time step)
a_time_anim = a_time(1:2:end);
traj_alpha_anim = traj_alpha(:,2:2:end);
my_biped.animateTrajectory(a_time_anim, traj_alpha_anim) 