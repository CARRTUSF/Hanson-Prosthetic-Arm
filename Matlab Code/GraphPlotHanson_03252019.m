% Carlo Canezo
% Hansen Arm Graph Plotting Spring 2019

function GraphPlotHanson_03252019(ManipW,ThetaDot,t)

% %Joint Angles Plot
% 
% figure
% plot(t,Theta,'linewidth',1);
% xlabel('time')
% ylabel('Joint Angles (radians)')
% title('Time vs Joint Angles')
% legend('JointR 1','JointR 2','JointR 3','JointR 4','JointR 5','JointR 6','JointR 7','location','northeastoutside')

%Joint Velocities Plot

figure
plot(t,ThetaDot,'linewidth',1);
xlabel('time')
ylabel('Joint Velocity')
title('Time vs Joint Velocity')
legend('Theta 1 dot','Theta 2 dot','Theta 3 dot','Theta 4 dot','Theta 5 dot','Theta 6 dot','Theta 7 dot','location','northeastoutside')

%Manipulability Plot

figure
plot(t,ManipW,'linewidth',1);
xlabel('time')
ylabel('Manipulability')
title('Time vs Manipulability')
legend('Manipulability','location','northeastoutside')