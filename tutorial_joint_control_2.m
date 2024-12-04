% Hamlyn Centre
% Imperial College London
% 2024 MRes in Medical Robotics and Instrumentation
% Tutorial week 5 example code
% Author: Junhong Chen

clc;
close all;
clear;

%% Robot Joint Space Control
% Make sure to have the add-on "ZMQ remote API"
% running in CoppeliaSim


% Check before requiring the 'sim' module
host = '127.0.0.1'; % CoppeliaSim IP
port = 23000;       % Port CoppeliaSim ZeroMQ is listening on
isSimLoaded = false; 
% Try connecting to the specified host and port

try
    tcpClient = tcpclient(host, port, 'Timeout', 5); % 5-second timeout
    fprintf('Connection to CoppeliaSim is successful.\n');
    clear tcpClient;
    isSimLoaded = true; 
catch
    error('Could not connect to CoppeliaSim. Ensure it is running and the port is open.');
end

fprintf('Program started\n')

% Initialize RemoteAPIClient and connect to CoppeliaSim
client = RemoteAPIClient();
sim = client.require('sim');
 
% Match the robot's joint names (modify as needed based on your robot model object alias in simulation)
Name_joints = {'/Mirobot/joint1', '/Mirobot/joint2', '/Mirobot/joint3', '/Mirobot/joint4', '/Mirobot/joint5', '/Mirobot/joint6'};
Name_Tip = '/Mirobot/Tip';
Num_joints = length(Name_joints);
% Retrieve handles for each joint
Handles_joints = zeros(Num_joints, 1);

for i = 1 : Num_joints
    Handles_joints(i) = sim.getObject(Name_joints{i});
end
Handles_tip = sim.getObject(Name_Tip);
% Run the simulation:
sim.startSimulation();
% Define target positions for each joint in radians
Target_positions = zeros(1,6);

% Calculating the joint motion path in joint space
dt = 0.05; % Time of each simulation step
executing_time = 10; % Total executing time of this motion in Seconds
starting_pos = [pi/4, 0, pi/4, 0, pi/2, 0];
ending_pos = [-pi/4, 0, -pi/6, pi/2, 0, 0];

% Interpolation from staring to end joint position with given running steps
t_step = 0:dt:executing_time; % Interpolated time axis
step_pos = interp1([0, executing_time],[starting_pos; ending_pos],t_step); % Linear interpolation

% Create an array to store robot tip position
tip_positions = zeros(length(t_step),3);

% Set robot joint start position in simulation

for i = 1:Num_joints
    sim.setJointPosition(Handles_joints(i), starting_pos(i));
end

disp('Jogging the robot by directly setting joint position');
for steps = 1:length(t_step)
    
    % Using joint control to target position
    for i = 1:Num_joints
        sim.setJointTargetPosition(Handles_joints(i), step_pos(steps,i));
    end
    % You can also try to set Joint Velocity in this api, get more details from manual book.  
    % Recording tip position
    Tip_pose_array = sim.getObjectPosition(Handles_tip);
    tip_positions(steps,1) = Tip_pose_array{1};
    tip_positions(steps,2) = Tip_pose_array{2};
    tip_positions(steps,3) = Tip_pose_array{3};
    sim.wait(dt); % Allow time for the simulator to run
end
disp("End jogging");

% Wait to allow the motion to complete
pause(2); % Adjust pause time as needed based on motion speed

sim.stopSimulation();
fprintf('Program ended\n');

%% Visualise the robot tip trajectory
% Drop the first point because of null data
tip_positions = tip_positions(2:end, :); 
% Extract X, Y, and Z coordinates from the trajectory array
X = tip_positions(:, 1);
Y = tip_positions(:, 2);
Z = tip_positions(:, 3);
% Arrange the plotting space
xlim([-0.35, 0.35]); 
ylim([-0.35, 0.35]); 
zlim([-0.8, 0.45]);
% Plot the trajectory as a 3D line
plot3(X, Y, Z, 'b', 'LineWidth', 2);
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
title('Trajectory Plot by Joint Control');
grid on;
