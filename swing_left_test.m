load('trajectory.mat')
new_traj = [traj_alpha(4:6,:); traj_alpha(1:3,:)]
my_biped = Biped();
my_link_lens = [2.00, 3.625, 3.625, 0.5, 1.75; 2.00, 3.625, 3.625, 0.5, 1.75];
my_biped.set_geometry(my_link_lens);
a_alpha = [0 0 0; 0 0 0];
my_biped.set_alpha(a_alpha);
my_biped.set_stance('RIGHT_FOOT')
my_biped.animateTrajectory(a_time, new_traj)
%a_time, new_traj are the time and trajectories you want for just this step