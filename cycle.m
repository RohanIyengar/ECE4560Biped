my_biped = Biped();
my_link_lens = [2.00, 3.625, 3.625, 0.5, 1.75; 2.00, 3.625, 3.625, 0.5, 1.75];
my_biped.set_geometry(my_link_lens);
a_alpha = [0 0 0; 0 0 0];
my_biped.set_alpha(a_alpha);

for i = 1:3
    my_biped.set_stance('LEFT_FOOT')

    load('single_right.mat');
    a_time = a_time .* 2;
    my_biped.animateTrajectory(a_time, traj_alpha)

    load('stand_right.mat');
    my_biped.animateTrajectory(a_time, traj_alpha)
    a_time = a_time .* 2;
    my_biped.set_alpha([traj_alpha(1,end), traj_alpha(2,end), traj_alpha(3,end); traj_alpha(4,end), traj_alpha(5,end), traj_alpha(6,end)]);
    [~, ~, all_frames] = my_biped.fwd_kinematics('LEFT_FOOT');
    right_foot_final = all_frames{8}.mat;
    my_biped.set_gRO(right_foot_final);

    my_biped.set_stance('RIGHT_FOOT')

    load('single_left.mat');
    a_time = a_time .* 2;
    my_biped.animateTrajectory(a_time, traj_alpha)

    load('stand_left.mat');
    my_biped.animateTrajectory(a_time, traj_alpha)
    my_biped.set_alpha([traj_alpha(1,end), traj_alpha(2,end), traj_alpha(3,end); traj_alpha(4,end), traj_alpha(5,end), traj_alpha(6,end)]);
    [~, ~, all_frames] = my_biped.fwd_kinematics('RIGHT_FOOT');
    left_foot_final = all_frames{8}.mat;
    my_biped.set_gRO(left_foot_final);
end

%Final position of left foot
my_biped.get_gRO()