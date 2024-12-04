function [] = video_simulate(joint_angles)

for i = 1 : size(joint_angles, 1) - 1
    Initial = joint_angles(i, :);
    Final = joint_angles(i+1, :);
    Step = 2;
    framesPerSecond = 15;
    Qd = Final - Initial;
    for j = 1:Step
        q = Initial + Qd / Step * j;
        show(robot,q,"Collisions","on","Frames","off");
        F(i * Step + j) = getframe(gcf) ;
        drawnow
    end
end

writerObj = VideoWriter('iikVideo.avi');
writerObj.FrameRate = 10;
open(writerObj);
for i=1:length(F)
    frame = F(i) ;    
    writeVideo(writerObj, frame);
end
close(writerObj);

end