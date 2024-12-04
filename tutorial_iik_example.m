% Hamlyn Centre
% Imperial College London
% 2024 MRes in Medical Robotics and Instrumentation
% Tutorial week 7 example code
% Author: Junhong Chen

clc;
close all;
clear;

%%   Setup Robot
% add a forth row to the DH table to calculate the end-effector position
% add a forth frame for setup robot

robot = rigidBodyTree("DataFormat","column");
base = robot.Base;
rotating = rigidBody("rotating_base");
arm1 = rigidBody("arm1");
arm2 = rigidBody("arm2");
endeffector = rigidBody("endeffector");

collrota = collisionCylinder(0.03,0.095);    % cylinder: radius,length
collrota.Pose = trvec2tform([0 0 -0.095/2]); % centre
coll1 = collisionBox(0.105,0.025,0.056);  % box:length,width,height (x,y,z)
coll1.Pose = trvec2tform([0.105/2 0 0]);  % centre

tribox = zeros(6,3); %6 by 3 mesh
tribox(1,:) = [0,  0.025/2,  0.056/2];
tribox(2,:) = [0, -0.025/2,  0.056/2];
tribox(3,:) = [0,  0.025/2, -0.056/2];
tribox(4,:) = [0, -0.025/2, -0.056/2];
tribox(5,:) = [0.1,  0.025/2, 0];
tribox(6,:) = [0.1, -0.025/2, 0];
collEndeffector = collisionMesh(tribox); % Mesh shape
collEndeffector.Pose = trvec2tform([0 0 0]); % centre

addCollision(rotating,collrota)           % add Collision to the links
addCollision(arm1,coll1)
addCollision(arm2,collEndeffector)

% set joints as revolute joints
jntrota = rigidBodyJoint("rotation_joint","revolute");    
jnt1 = rigidBodyJoint("jnt1","revolute");
jnt2 = rigidBodyJoint("jnt2","revolute");
jntend = rigidBodyJoint("endeffector_joint","revolute");


%          a        alpha   d       theta      modified DH table 
dhparams = [0   	0    	0.095  	0;
            0    	pi/2    0       0;
            0.105  -pi/2    0       0;
            0.1     0       0       0];
       
bodies = {base,rotating,arm1,arm2,endeffector};  
joints = {[],jntrota,jnt1,jnt2,jntend};

figure("Name","Robot","Visible","on")               % Visualise the robot
for i = 2:length(bodies)
    setFixedTransform(joints{i},dhparams(i-1,:),"mdh");
    bodies{i}.Joint = joints{i};
    addBody(robot,bodies{i},bodies{i-1}.Name)
    show(robot,"Collisions","on","Frames","off");
    drawnow;     
end
figure;
show(robot)

%% Iterative Inverse Kinematics

%          a        alpha   d       theta      modified DH table 
dhparams = [0   	0    	0.095  	0;
            0    	pi/2    0       0;
            0.105  -pi/2    0       0;
            0.1     0       0       0];

homeconfig = homeConfiguration(robot);    % Setting initial angle
finalconfig = [pi/4, pi/4, pi/4, 0]';     % Setting final angle as reference

syms th1 th2 th3   % Do not forget to install "Symbolic Math Toolbox"
d1 = 0.095;
a2 = 0.105;
a3 = 0.1;

T01 = [cos(th1), -sin(th1), 0,     0;      % Transformation Matrix that transforms vectors defined in frame 1 to their description in base frame 0
       sin(th1),  cos(th1), 0,     0;
       0,         0.        1,     d1;
       0,         0,        0,     1];
   
T12 = [cos(th2), -sin(th2), 0,     0;      % Transformation Matrix that transforms vectors defined in frame 2 to their description in frame 1
       0,         0,       -1,     0;
       sin(th2),  cos(th2)  0,     0;
       0,         0,        0,     1];
   
T23 = [cos(th3), -sin(th3), 0,     a2;      % Transformation Matrix that transforms vectors defined in frame 3 to their description in frame 2
       0,         0,        1,     0;
      -sin(th3), -cos(th3), 0,     0;
       0,         0,        0,     1];
   
T34 = [1,         0,        0,     a3;    % Transformation Matrix that transforms vectors defined in frame 4 to their description in frame 3 (th4 = 0)
       0,         1,        0,     0;
       0,         0.        1,     0;
       0,         0,        0,     1];
   
T04 = T01 * T12 * T23 * T34;
T = simplify(T04);                       % Transformation Matrix that transforms vectors defined in frame 4 to their description in base frame 0

%% Computing the desired end-effector position

Q_t = T(1:3,4);                               % Translation Vector
JQ = jacobian(Q_t,[th1,th2,th3]);             % Jacobian Matrix  3 by 3

P1 = subs(Q_t,th1,finalconfig(1));            % Substitute each angle from symbol to number in initial condition. 
P2 = subs(P1,th2,finalconfig(2));
P3 = subs(P2,th3,finalconfig(3));
p_target = double(P3);                       % Calculating the final position from given final angle, convert from symbolic type to double type.

theta_k_1 = homeconfig(1:3);                   % Initial interative, note that since you have four rows, the homeconfig is a 4 by 1 vector, we only need first 3 elements
theta_list = theta_k_1;                 


%% Newton Raphson's loop

theta = [-pi/2, pi/2;      %Angle constraints
             0,   pi;
         -pi/2, pi/2];

Q1 = subs(Q_t,th1,theta_k_1(1));
Q2 = subs(Q1,th2,theta_k_1(2));
Q3 = subs(Q2,th3,theta_k_1(3));
Q_k_1 = double(vpa(Q3,12)); % Translation vector for theta_k_1, Convert from symbolic to numeric 

Vp_to_target = p_target-Q_k_1; % vector from guessed position to desired target position 
p_to_target = norm(Vp_to_target); % Distance between target position and guess 


while p_to_target > 0.0001
    
    J0 = subs(JQ,th1,theta_k_1(1));
    J1 = subs(J0,th2,theta_k_1(2));
    J2 = subs(J1,th3,theta_k_1(3));
    Jacob = double(vpa(J2,12));       % Jacobian for theta_k_1, Convert from symbolic to numeric       
    Jinv = pinv(Jacob);

    theta_k = theta_k_1 + Jinv * Vp_to_target;     % Using the iterative method
    
    for i = 1:size(theta_k,1)                                        % Angle Constrains 
        theta_k(i) = max(min(theta_k(i), theta(i,2)),theta(i,1));    
    end
    
    theta_list = [theta_list, theta_k];     

    Q1 = subs(Q_t,th1,theta_k(1));
    Q2 = subs(Q1,th2,theta_k(2));
    Q3 = subs(Q2,th3,theta_k(3));
    Q_k = double(vpa(Q3,12));         % Translation vector for newly computed theta_k, Convert from symbolic to numeric   
       
    Vp_to_target = p_target-Q_k;         % vector from estimated position to desired target position
    p_to_target = norm(Vp_to_target);     
    
    theta_k_1 = theta_k; 
    Q_k_1 = Q_k;   
end

theta_final = [theta_list(:,end); 0];      % We need to add one row to visualize in Matlab.

%% Robot Simulation & Creating Video
Initial = homeconfig;
Final = theta_final;
Step = 10;
framesPerSecond = 15;
Qd = Final - Initial;
for i = 1:Step
    q = Initial + Qd / Step * i;
    show(robot,q,"Collisions","on","Frames","off");
    F(i) = getframe(gcf) ;
    drawnow
end

% create the video writer with 1 fps
writerObj = VideoWriter('iikVideo.avi');
writerObj.FrameRate = 10;
% set the seconds per image
% open the video writer
open(writerObj);
% write the frames to the video
for i=1:length(F)
    % convert the image to a frame
    frame = F(i) ;    
    writeVideo(writerObj, frame);
end
% close the writer object
close(writerObj);

