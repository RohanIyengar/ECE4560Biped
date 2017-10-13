% Instantiate instance/object of the Biped() class
my_biped = Biped();

% Week 2 Biped Script 
my_link_lens = [2.00, 3.625, 3.625, 0.5, 1.75; 2.25, 3.625, 3.625, 0.5, 1.75];
my_biped.set_geometry( my_link_lens );


% Joint configuration #3
my_joint_angles_3 = [pi/3, -pi/2, 0; -pi/3, -pi/2, 0];
my_biped.set_alpha( my_joint_angles_3 );
figure
hold on
my_biped.plotTF2();
figure
my_biped.set_stance('LEFT_FOOT')
hold on
my_biped.plotTF2();
figure
my_biped.set_stance('RIGHT_FOOT')
hold on
my_biped.plotTF2();
hold on