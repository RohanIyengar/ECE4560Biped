% Instantiate instance/object of the Biped() class
my_biped = Biped();

% Week 4 Biped Script 
my_link_lens = [2.00, 3.625, 3.625, 0.5, 1.75; 2.00, 3.625, 3.625, 0.5, 1.75];
my_biped.set_geometry( my_link_lens );
a_alpha = [pi/2 -pi/3 pi/5; 0 0 0];
my_biped.set_alpha(a_alpha);

%Calculate Jacobian
jacobian1 = my_biped.jacobian(a_alpha, 'TORSO', 'LEFT_FOOT');

%Get trajectory
[g_t_lf, g_t_rf] = my_biped.fk_torso_foot();
m = g_t_lf.getM();
xi = m(1,3);
yi = m(2,3);
thetai = acos(m(1,1));
dt = 0.2;
a_time = 0.1:dt:5;
xvec = linspace(xi, xi-1, length(a_time));
yvec = linspace(yi, yi+1, length(a_time));
thetavec = linspace(thetai, thetai - pi, length(a_time));

a_traj_ws = [xvec; yvec; thetavec];

traj_alpha = my_biped.rr_inv_kin(a_traj_ws, a_time, 'TORSO', 'LEFT_FOOT');
traj_alpha = [traj_alpha; zeros(3,length(a_time))];

%rr_inv_kin angles
figure
subplot(3,1,1)
plot(a_time, traj_alpha(1,:));
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

%x-y position over time
figure
title('Desired position');
subplot(2,1,1)
plot(a_time, xvec);
xlabel('Time (s)');
ylabel('X position (in)');
axis([0 5 3 6]);
subplot(2,1,2)
plot(a_time, yvec);
xlabel('Time (s)');
ylabel('Y position (in)');
axis([0 5 -6 -3]);

%Accomplished trajectory
newx = zeros(length(a_time));
newy = zeros(length(a_time));

for i = 1:length(a_time);
    my_biped.set_alpha([traj_alpha(1:3,i)'; traj_alpha(4:6,i)']);
    [g_t_lf, g_t_rf] = my_biped.fk_torso_foot();
    m = g_t_lf.getM();
    newx(i) = m(1,3);
    newy(i) = m(2,3);
end
figure
title('Accomplished trajectory')
subplot(2,1,1)
plot(a_time, newx);
xlabel('Time (s)');
ylabel('X position (in)');
axis([0 5 3 6]);
subplot(2,1,2)
plot(a_time, newy);
xlabel('Time (s)');
ylabel('Y position (in)');
axis([0 5 -6 -3]);

%Animation
my_biped.animateTrajectory(a_time, traj_alpha)