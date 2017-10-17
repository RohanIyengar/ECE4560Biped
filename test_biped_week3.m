% Instantiate instance/object of the Biped() class
my_biped = Biped();

% Week 3 Biped Script 
my_link_lens = [2.00, 3.625, 3.625, 0.5, 1.75; 2.00, 3.625, 3.625, 0.5, 1.75];
my_biped.set_geometry( my_link_lens );

%% Part 1
% Link mass defaults to 53.5 g. CoM is calculated by summing the position
% of each motor in a desired frame multiplied by the motor's mass, and
% finally dividing by total mass of the motors.

%% Part 2

% Joint configuration #3
% Figure 1 has the torso POV, 2 has left foot POV, 3 has 
my_joint_angles_3 = [pi/3, -pi/2, 0; -pi/3, -pi/2, 0];
my_biped.set_alpha( my_joint_angles_3 );
figure
hold on
my_biped.plotTF2();
figure
my_joint_angles_1 = [pi/2, -pi/2, -pi/4; 0, 0, 0];
my_biped.set_alpha(my_joint_angles_1);
my_biped.set_stance('LEFT_FOOT');
hold on
my_biped.plotTF2();
figure
my_biped.set_stance('RIGHT_FOOT');
my_joint_angles_2 = [3*pi/4, 0, 0; 0, -pi/4, 0];
my_biped.set_alpha( my_joint_angles_2 );
hold on
my_biped.plotTF2();
hold on

%% Part3
a_time = 0.0:0.15:7.35
anglesfwd = [linspace(pi/3, -pi/3, 10); linspace(-pi/2, pi/2, 10); linspace(0,0,10); linspace(-pi/3, pi/3, 10); linspace(-pi/2, pi/2, 10); linspace(0, 0, 10)];
angles = [anglesfwd, -anglesfwd, anglesfwd, -anglesfwd, anglesfwd];
% Animation from the torso point of view
my_biped.set_stance('TORSO')
my_biped.animateTrajectory(a_time, angles);

%% Part4
% A support polygon is a region where the center of mass must lie for an
% object to be stable. This is important for bipedal walking because if the
% center of mass moves outside of this point, the robot will fall over. If
% the robot is not moving and has one foot on the ground, the center of
% mass will need to be within a small area directly above the foot. If both
% feet are on the ground, the center of mass needs to be between the two
% feet. If the robot is moving, this polygon changes and can depend on the
% robot's momentum.