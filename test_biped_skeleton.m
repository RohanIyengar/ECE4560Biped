%========================== test_biped_skeleton.m =========================
%
%  Brief demo of skeleton Biped() class
%
%========================== test_biped_skeleton.m =========================

% Instantiate instance/object of the Biped() class
my_biped = Biped();

% Example Biped class method calls - 
%       Note: these don't do anything as the methods are all stubs  
%             waiting to be filled out by you
my_link_lens = [2.25, 3.625, 3.625, 0.5, 1.75; 2.25, 3.625, 3.625, 0.5, 1.75];
my_biped.set_geometry( my_link_lens );

% Joint configuration #1
my_joint_angles_1 = [pi/2, -pi/2, -pi/4; 0, 0, 0];
my_biped.set_alpha( my_joint_angles_1 );

[my_g_t_lf, my_g_t_rf] = my_biped.fk_torso_foot();
    
my_biped.plotTF();

% Joint configuration #2
my_joint_angles_2 = [3*pi/4, 0, 0; 0, -pi/4, 0];
my_biped.set_alpha( my_joint_angles_2 );

[my_g_t_lf, my_g_t_rf] = my_biped.fk_torso_foot();
    
my_biped.plotTF();

% Joint configuration #3
my_joint_angles_3 = [pi/3, -pi/2, 0; -pi/3, -pi/2, 0];
my_biped.set_alpha( my_joint_angles_3 );

[my_g_t_lf, my_g_t_rf] = my_biped.fk_torso_foot();
    
my_biped.plotTF();

