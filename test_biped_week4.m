% Instantiate instance/object of the Biped() class
my_biped = Biped();

% Week 4 Biped Script 
my_link_lens = [2.00, 3.625, 3.625, 0.5, 1.75; 2.00, 3.625, 3.625, 0.5, 1.75];
my_biped.set_geometry( my_link_lens );
a_alpha = [pi/2 -pi/3 pi/5; 0 0 0];
my_biped.set_alpha(a_alpha);

%my_biped.plotTF();

%Calculate Jacobian
jacobian1 = my_biped.jacobian(a_alpha, 'TORSO', 'LEFT_FOOT');

%Get trajectory
[g_t_lf, g_t_rf] = my_biped.fk_torso_foot();
m = g_t_lf.getM();
xi = m(1,3);
yi = m(2,3);
thetai = acos(m(1,1));
dt = 0.1;
a_time = 1:dt:5;
xvec = linspace(xi, xi-1, length(a_time));
yvec = linspace(yi, yi+1, length(a_time));
thetavec = linspace(thetai, thetai - pi/3, length(a_time));
a_traj_ws = [xvec; yvec; thetavec];

traj_alpha = my_biped.rr_inv_kin(a_traj_ws, a_time, 'TORSO', 'LEFT_FOOT');