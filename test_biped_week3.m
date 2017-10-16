% Instantiate instance/object of the Biped() class
my_biped = Biped();

% Week 3 Biped Script 
my_link_lens = [2.00, 3.625, 3.625, 0.5, 1.75; 2.00, 3.625, 3.625, 0.5, 1.75];
my_biped.set_geometry( my_link_lens );


% Joint configuration #1
% Figure 1 has the torso POV, 2 has left foot POV, 3 has 
my_joint_angles_3 = [pi/3, -pi/2, 0; -pi/3, -pi/2, 0];
my_biped.set_alpha( my_joint_angles_3 );
my_biped.set_stance('RIGHT_FOOT');
figure
hold on
my_biped.plotTF2();
pos_com = my_biped.com('RIGHT_FOOT')