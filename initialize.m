%%
clc
close all
clear s
myObj = Mirobot_Matlab; 
s = myObj.Mirobotconnect('/dev/tty.usbserial-10');
disp('Finish');
pause(2);

%%
disp('Zeroing');
myObj.go_to_zero(s); %return back to initial position
pause(2);
disp('Initialing');
myObj.go_to_axis(s,0,-30,35,0,-90,0); %go to axis positoin
pause(2);
disp('Finish');