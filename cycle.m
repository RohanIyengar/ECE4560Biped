my_biped = Biped();
my_link_lens = [2.00, 3.625, 3.625, 0.5, 1.75; 2.00, 3.625, 3.625, 0.5, 1.75];
my_biped.set_geometry(my_link_lens);
a_alpha = [0 0 0; 0 0 0];
my_biped.set_alpha(a_alpha);

my_biped.set_stance('LEFT_FOOT')

load('single_right.mat');
my_biped.animateTrajectory(a_time, traj_alpha)

load('stand_right.mat');
my_biped.animateTrajectory(a_time, traj_alpha)

my_biped.set_stance('RIGHT_FOOT')

load('single_left.mat');
my_biped.animateTrajectory(a_time, traj_alpha)

load('stand_left.mat');
my_biped.animateTrajectory(a_time, traj_alpha)
