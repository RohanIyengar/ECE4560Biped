% Instantiate instance/object of the Biped() class
my_biped = Biped();

% Week 4 Biped Script 
my_link_lens = [2.00, 3.625, 3.625, 0.5, 1.75; 2.00, 3.625, 3.625, 0.5, 1.75];
my_biped.set_geometry( my_link_lens );
a_alpha = [pi/2; -pi/3; pi/5];
jacobian1 = my_biped.jacobian(a_alpha, 'TORSO', 'LEFT_FOOT')