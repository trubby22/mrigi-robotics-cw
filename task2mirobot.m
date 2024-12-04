clc;
close all;
clear;

%% notes


initialandfinal = [rad2deg(pi/4), 0, rad2deg(-pi/4), rad2deg(-pi/5), 0, 0 ];
pose1 = [0, rad2deg(-pi/3), rad2deg(pi/5), 0, 0, 0 ];
pose2 = [rad2deg(-pi/4), rad2deg(pi/3), rad2deg(-pi/6), rad2deg(pi/3), 0, 0 ];

%%
clc
close all
clear s


myObj = Mirobot_Matlab; 
s = myObj.Mirobotconnect('/dev/cu.usbserial-10'); 

fprintf('started')

myObj.go_to_zero(s); %return back to zero position
pause(2);


for i = 1:6
    myObj.move_to_axis(s,i,'ccw',initialandfinal(1,i));
    pause(1);
end

for i = 1:6
    myObj.move_to_axis(s,i,'ccw',pose1(1,i));
    pause(1);
end

for i = 1:6
    myObj.move_to_axis(s,i,'ccw',pose2(1,i));
    pause(1);
end

for i = 1:6
    myObj.move_to_axis(s,i,'ccw',initialandfinal(1,i));
    pause(1);
end






%%

