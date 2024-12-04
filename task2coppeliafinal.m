jointnames = {'/Mirobot/joint1', '/Mirobot/joint2', '/Mirobot/joint3', '/Mirobot/joint4', '/Mirobot/joint5', '/Mirobot/joint6'};
numberofjoints = length(jointnames);
jointhandles = zeros(numberofjoints, 1);

for i = 1 : numberofjoints
    jointhandles(i) = sim.getObject(jointnames{i});
end


endEffectorHandle = sim.getObject('/Mirobot/Tip'); 


endEffectorX = [];
endEffectorY = [];
endEffectorZ = [];


updateTrajectory = @(handle) cell2mat(sim.getObjectPosition(handle, -1));

sim.startSimulation();

startandendposition = [-pi/4 0 pi/4 pi/5 0 0];
targetposition1 = [0 pi/3 -pi/5 0 0 0];
targetposition2 = [-pi/4 pi/3 -pi/6 pi/3 0 0];

increment = 0.01; 
pausetime = 0.02; 

movetoposition = @(current, target) (current + sign(target - current) * min(abs(target - current), increment));

currentposition = startandendposition;


while any(abs(currentposition - startandendposition) > increment)
    for i = 1:length(jointhandles)
        currentposition(i) = movetoposition(currentposition(i), startandendposition(i));
        sim.setJointTargetPosition(jointhandles(i), currentposition(i));
    end
    position = updateTrajectory(endEffectorHandle);
    endEffectorX(end + 1) = position(1);
    endEffectorY(end + 1) = position(2);
    endEffectorZ(end + 1) = position(3);
    pause(pausetime);
end


while any(abs(currentposition - targetposition1) > increment)
    for i = 1:length(jointhandles)
        currentposition(i) = movetoposition(currentposition(i), targetposition1(i));
        sim.setJointTargetPosition(jointhandles(i), currentposition(i));
    end
    position = updateTrajectory(endEffectorHandle);
    endEffectorX(end + 1) = position(1);
    endEffectorY(end + 1) = position(2);
    endEffectorZ(end + 1) = position(3);
    pause(pausetime);
end

while any(abs(currentposition - targetposition2) > increment)
    for i = 1:length(jointhandles)
        currentposition(i) = movetoposition(currentposition(i), targetposition2(i));
        sim.setJointTargetPosition(jointhandles(i), currentposition(i));
    end
    position = updateTrajectory(endEffectorHandle);
    endEffectorX(end + 1) = position(1);
    endEffectorY(end + 1) = position(2);
    endEffectorZ(end + 1) = position(3);
    pause(pausetime);
end

while any(abs(currentposition - startandendposition) > increment)
    for i = 1:length(jointhandles)
        currentposition(i) = movetoposition(currentposition(i), startandendposition(i));
        sim.setJointTargetPosition(jointhandles(i), currentposition(i));
    end
    position = updateTrajectory(endEffectorHandle);
    endEffectorX(end + 1) = position(1);
    endEffectorY(end + 1) = position(2);
    endEffectorZ(end + 1) = position(3);
    pause(pausetime);
end

sim.stopSimulation();


figure;
plot3(endEffectorX, endEffectorY, endEffectorZ, 'b-', 'LineWidth', 1.5);
hold on;
scatter3(endEffectorX(1), endEffectorY(1), endEffectorZ(1), 50, 'g', 'filled'); % Starting point
scatter3(endEffectorX(end), endEffectorY(end), endEffectorZ(end), 50, 'r', 'filled'); % Ending point
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
grid on;
title('End effector trajectory');
legend('trajectory', 'start', 'end');

