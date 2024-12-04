%%
clc
close all
clear s
myObj = Mirobot_Matlab; 
s = myObj.Mirobotconnect('/dev/tty.usbserial-14430'); %connect to /dev/tty.usbserial-10 for example
disp('Finish');
pause(2);

%%
disp('Testing zeroing');
myObj.go_to_zero(s); %return back to initial position
pause(2);


%%
disp('Testing joint control');
for i = 1:6 %each axis turn counterclockwise and then clockwise by 30 degree
    myObj.move_to_axis(s,i,"ccw",30);
    pause(2);
    myObj.move_to_axis(s,i,"cw",30);
    pause(2);
    
end

%%
disp('Testing coordinate control');
myObj.go_to_cartesian_lin(s,240,0,220,0,0,0); %go to cartesian position
pause(2);

%%
disp('Testing angle step control');
myObj.increment_axis(s,10,0,0,0,0,0); %increment axis 1 by 10
pause(2);

%%
disp('Testing target control');
myObj.go_to_axis(s,-45,0,0,-60,0,0); %go to axis positoin
pause(2);

%%
disp('Testing coordinate step control');
myObj.increment_cartesian_lin(s,10,0,0,0,0,0); %increment by 10
pause(2);

%%
disp('Testing direction control');
myObj.direction_mobility(s,"right",40); %move to right by 40
pause(2);

%%
disp('Zeroing');
myObj.go_to_zero(s); %return back to initial position
pause(2);


%%
disp('Testing continuous coordinate step control');
a = -2;                           %increment z by -1 each time for 30 steps
for i=1:30
    myObj.increment_cartesian_lin(s,0,0,a,0,0,0);
    pause(0.02);
    i
end

%%
disp('Testing continuous angle step control');
b = -1;                            %increment axis 1,2,3 by -1 each time for 30 steps
for i=1:30
    myObj.increment_axis(s,b,b,b,0,0,0);
    pause(0.02);
    i
end

%%
disp('Zeroing');
myObj.go_to_zero(s); %return back to initial position
pause(2);