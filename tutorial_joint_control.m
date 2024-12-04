% Hamlyn Centre
% Imperial College London
% 2024 MRes in Medical Robotics and Instrumentation
% Tutorial week 5 example code
% Author: Junhong Chen

clc;
close all;
clear;

%%
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
Num_joints = length(Name_joints);
Num_steps = 500;
% Retrieve handles for each joint
Handles_joints = zeros(Num_joints, 1);

for i = 1 : Num_joints
    Handles_joints(i) = sim.getObject(Name_joints{i});
end

% Run the simulation:
sim.startSimulation();
% Define target positions for each joint in radians
Target_positions = zeros(1,6);

% Set robot joint position in simulation
% Set robot joint position in simulation
disp('Jogging the robot by directly setting joint position');
for steps = 1:Num_steps
    % Produce a sinusoidal variable for joint 3 and joint 1
    Target_positions = [1 * sin(0.01 * steps), 0, 1 * sin(0.01 * steps), 0, 0, 0];

    % Using joint control to target position
    for i = 1:Num_joints
        sim.setJointTargetPosition(Handles_joints(i), Target_positions(i));
    end
    % You can also try to set Joint Velocity in this api, get more details from manual book.    
    
    sim.wait(0.05); % Allow time for the simulator to run
end
disp("End jogging");

% Wait to allow the motion to complete
pause(2); % Adjust pause time as needed based on motion speed

sim.stopSimulation();
fprintf('Program ended\n');