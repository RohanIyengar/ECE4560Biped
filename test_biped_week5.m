% Instantiate instance/object of the Biped() class
my_biped = Biped();

% Week 4 Biped Script 
my_link_lens = [2.00, 3.625, 3.625, 0.5, 1.75; 2.00, 3.625, 3.625, 0.5, 1.75];
my_biped.set_geometry( my_link_lens );
a_alpha = [0 0 0; 0 0 0];
my_biped.set_alpha(a_alpha);

%Animation
my_biped.animateTrajectory(a_time, traj_alpha)