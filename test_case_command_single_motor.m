%% Test Case: Command Single Motor
% Commands motor IDs 10 and 11 to identical angular positions. Both motors
% will actuate at identical speeds.

%% Pre-conditions
% Cycle power on OpenCM9.04 controller

%% Add Dynamixel_IO to classpath
MATLIBS = '../../Dynamixel_IO/';
addpath( MATLIBS );

%% Initialize Dynamixel_IO
dxl_io = Dynamixel_IO;  % generate instance of the Dynamixel_IO class
dxl_io.load_library();  % load library appropriate to OS (auto-detected)
dxl_io.connect(0, 1);   % connect to port 0, at 1 MBaud

motor_ids = [9 7 2 6 8 3];      % list of motor IDs to command

%% Test Case - Trajectory: Goal Position/Speed
% dest_pos = pi/2;                   % destination position
% dest_speed = dest_pos/1;    % speed
% 
% input('Press <Enter> to begin goal position/speed test.\n');
% % 1st arg: list of motor IDs, 2nd arg: list of joint biases, 
% % 3rd arg: destination motor positions, 4th arg: motor speeds
% dxl_io.set_motor_pos_speed(motor_ids, zeros(length(motor_ids), 1), dest_pos, dest_speed );
% 

%% Next challenge: 
% Can you modify the above code to command just motor ID 10 to smoothly 
% execute a sinusoidal trajectory of the form A*sin(2*pi*f*t), where A =
% amplitude is radians, f = frequency (Hz) and t is the current time??
load trajectory.mat
dt = a_time(2);
for i = 2:length(a_time)
    dest_pos = traj_alpha(:,i);
    prev_pos = traj_alpha(:,i-1);
    dest_speed = (dest_pos - prev_pos) ./ dt;
    dxl_io.set_motor_pos_speed(motor_ids, zeros(length(motor_ids), 1), dest_pos, dest_speed );
    pause(dt);
end
