% Carlo Canezo
% Hansen Arm Main Code Spring 2019
clear all;
clc;
global DH
global dHo

dHo = [0;0;0;0;0;0;0];

% Transformation Matrices Below are user Input


Ti=[0.0865   -0.9962    0.0065    6.5929;
   -0.9962   -0.0865    0.0085  -53.3749;
   -0.0079   -0.0072   -0.9999    3.2465;
         0         0         0    1.0000]
 
Tf=[0.9700    0.2405    0.0356    45.2219;
   -0.2280    0.9508   -0.2099    50;
   -0.0843    0.1955    0.9771    -20;
         0         0         0    1.0000]
     
% Tf=[0.9700    0.2405    0.0356    -25.2219;
%    -0.2280    0.9508   -0.2099    0;
%    -0.0843    0.1955    0.9771    0;
%          0         0         0    1.0000]
     
 % Q initial for Initial Forward Kinematics Coverted to Radians

% TH1=90*(pi/180);
% TH2=90*(pi/180);
% TH3=-90*(pi/180);
% TH4=0*(pi/180);
% TH5=-90*(pi/180);
% TH6=-90*(pi/180);
% TH7=0*(pi/180);

TH1=5*(pi/180);
TH2=5*(pi/180);
TH3=5*(pi/180);
TH4=5*(pi/180);
TH5=5*(pi/180);
TH6=5*(pi/180);
TH7=5*(pi/180);


% User Input Matrices Verification

s=size(Ti);
if (s(1) ~= 4 || s(2) ~= 4);
    fprintf('\n ERROR! Your Ti is not 4X4 -recheck-');
    return
end

p=size(Tf);
if (p(1) ~= 4 || s(2) ~= 4);
    fprintf('\n ERROR! Your Tf is not 4X4 -recheck-');
    return
end

s=size(Ti);
if (s(1) ~= 4 || s(2) ~= 4);
    fprintf('\n ERROR! Your Ti is not 4X4');
    return
end

if (any(Ti(4,1:3))==1);
    fprintf('\n ERROR! Matrix Values Incorrect Check T(4,1:3)')
    return
elseif (Ti(4,4)~=1);
    fprintf('\n ERROR! Matrix Values Incorrect Check T(4,4)')
    return
else disp('Transformation Matrix Format Ti Acceptable');
    
end

if (any(Tf(4,1:3))==1);
    fprintf('\n ERROR! Matrix Values Incorrect Check Tf(4,1:3)')
    return
elseif (Tf(4,4)~=1);
    fprintf('\n ERROR! Matrix Values Incorrect Check Tf(4,4)')
    return
else disp('Transformation Matrix Format Tf Acceptable');
    
end

% Vector formation Ti
RxvecTi=[Ti(1,1); Ti(2,1); Ti(3,1)];
RyvecTi=[Ti(1,2); Ti(2,2); Ti(3,2)];
RzvecTi=[Ti(1,3); Ti(2,3); Ti(3,3)];

% Ortho Check Ti
OrthoXYTi=RxvecTi'*RyvecTi;
OrthoXZTi=RxvecTi'*RzvecTi;
OrthoYZTi=RyvecTi'*RzvecTi;

% Unity Check Ti
AbsMagXvecTi=norm(RxvecTi);
AbsMagYvecTi=norm(RyvecTi);
AbsMagZvecTi=norm(RzvecTi);

% Rounded Orthocheck 5 decimal places Ti
XYTi=OrthoXYTi;
XZTi=OrthoXZTi;
YZTi=OrthoYZTi;

% Rounded Unity Check 5 decimal Places Ti
MagXTi=vpa(AbsMagXvecTi,3);
MagYTi=vpa(AbsMagYvecTi,3);
MagZTi=vpa(AbsMagZvecTi,3);

if (MagXTi < .99 || MagXTi > 1.1)
  fprintf('\n !CHECK MATRIX! Unit Vectors not detected for Ti');
  return
elseif (MagYTi < .99 || MagXTi > 1.1)
    fprintf('\n !CHECK MATRIX! Unit Vectors not detected for Ti');
  return
elseif (MagYTi < .99 || MagXTi > 1.1)
    fprintf('\n !CHECK MATRIX! Unit Vectors not detected for Ti');
  return
end  

if (abs(XYTi) <= 0.000 || abs(XZTi) <= 0.000 || abs(YZTi) <= 0.000)
fprintf('\n !CHECK MATRIX! Your Ti matrix is currently not orthogonal');
return
end

% Vector formation Tf
RxvecTf=[Tf(1,1); Tf(2,1); Tf(3,1)];
RyvecTf=[Tf(1,2); Tf(2,2); Tf(3,2)];
RzvecTf=[Tf(1,3); Tf(2,3); Tf(3,3)];

% Ortho Check Tf
OrthoXYTf=RxvecTf'*RyvecTf;
OrthoXZTf=RxvecTf'*RzvecTf;
OrthoYZTf=RyvecTf'*RzvecTf;


% Unity Check Tf
AbsMagXvecTf=norm(RxvecTf);
AbsMagYvecTf=norm(RyvecTf);
AbsMagZvecTf=norm(RzvecTf);


% Rounded Orthocheck 5 decimal places Tf
XYTf=OrthoXYTf;
XZTf=OrthoXZTf;
YZTf=OrthoYZTf;


% Rounded Unity Check 5 decimal Places Ti
MagXTf=vpa(AbsMagXvecTf,3);
MagYTf=vpa(AbsMagYvecTf,3);
MagZTf=vpa(AbsMagZvecTf,3);

if (MagXTf < .99 || MagXTf > 1.1)
  fprintf('\n !CHECK MATRIX! Unit Vectors not detected for Tf');
  return
elseif (MagYTf < .99 || MagXTf > 1.1)
    fprintf('\n !CHECK MATRIX! Unit Vectors not detected for Tf');
  return
elseif (MagYTf < .99 || MagXTf > 1.1)
    fprintf('\n !CHECK MATRIX! Unit Vectors not detected for Tf');
  return
end  

if (abs(XYTf) <= 0.000 || abs(XZTf) <= 0.000 || abs(YZTf) <= 0.000)
fprintf('\n !CHECK MATRIX! Your Ti matrix is currently not orthogonal');
return
end


%Time step
dt = 1/60;

t = 2;


%Number of Steps
n = floor(t/dt);

% Initial DH Callout

[DH]=DHArmHanson_03252019(TH1,TH2,TH3,TH4,TH5,TH6,TH7);

% Trajectory
% Acquire Initial Position Values from Tinitial
% Acquire Final Position Values from Tfinal

Pf=Tf(1:3,4);
Pi=Ti(1:3,4);
Rotf=Tf(1:3,1:3);
Roti=Ti(1:3,1:3);

% Obtain Trajectory Matrix Position & Orientation
[Tarm, traj]=TrajectoryHanson_03252019(Rotf,Roti,Pf,Pi,n);

% Transformation Matrices
% Will include:
% Initial Transformation Matrices with Respect to Zero
% Initial Position Vectors respect to each frames
% Initial Position Vectors respect to Zero

[P0_1, P0_2, P0_3, P0_4, P0_5, P0_6, P0_7, P0_8, T0_1, T0_2, T0_3, T0_4, T0_5, T0_6, T0_7, T0_8, P1_2, P2_3, P3_4, P4_5, P5_6, P6_7, P7_8, T1_2, T2_3, T3_4, T4_5, T5_6, T6_7, T7_8]=ForwardKinematicsHanson_03252019(TH1,TH2,TH3,TH4,TH5,TH6,TH7);

% Initial Position of Robot Links

XLink1=[0 P0_1(1)];
YLink1=[0 P0_1(2)];
ZLink1=[0 P0_1(3)];

XLink2=[P0_1(1) P0_2(1)];
YLink2=[P0_1(2) P0_2(2)];
ZLink2=[P0_1(3) P0_2(3)];

XLink3=[P0_2(1) P0_3(1)];
YLink3=[P0_2(2) P0_3(2)];
ZLink3=[P0_2(3) P0_3(3)];

XLink4=[P0_3(1) P0_4(1)];
YLink4=[P0_3(2) P0_4(2)];
ZLink4=[P0_3(3) P0_4(3)];

XLink5=[P0_4(1) P0_5(1)];
YLink5=[P0_4(2) P0_5(2)];
ZLink5=[P0_4(3) P0_5(3)];

XLink6=[P0_5(1) P0_6(1)];
YLink6=[P0_5(2) P0_6(2)];
ZLink6=[P0_5(3) P0_6(3)];

XLink7=[P0_6(1) P0_7(1)];
YLink7=[P0_6(2) P0_7(2)];
ZLink7=[P0_6(3) P0_7(3)];

XLink8=[P0_7(1) P0_8(1)];
YLink8=[P0_7(2) P0_8(2)];
ZLink8=[P0_7(3) P0_8(3)];

figure
hold on
RLink1 = plot3(XLink1, ZLink1, YLink1, 'r', 'linewidth', 3);
RLink2 = plot3(XLink2, ZLink2, YLink2, 'g', 'linewidth', 3);
RLink3 = plot3(XLink3, ZLink3, YLink3, 'b', 'linewidth', 3);
RLink4 = plot3(XLink4, ZLink4, YLink4, 'c', 'linewidth', 3);
RLink5 = plot3(XLink5, ZLink5, YLink5, 'm', 'linewidth', 3);
RLink6 = plot3(XLink6, ZLink6, YLink6, 'y', 'linewidth', 3);
RLink7 = plot3(XLink7, ZLink7, YLink7, 'b', 'linewidth', 3);
RLink8 = plot3(XLink8, ZLink8, YLink8, 'r', 'linewidth', 3);
RPath = plot3(traj(1:n,1),traj(1:n,3),traj(1:n,2),'--g', 'linewidth',1);
xlabel('X_position')
ylabel('Z_position')
zlabel('Y_position')
title('Robot Position')
grid off
axis([-60 60 -60 60 -60 60]);
hold off
view(-45, -15);

% Preallocation of Graphed Variables for Speed
% t = zeros(n+1,1);
% Theta = zeros(n+1, 3);
% ThetaDot = zeros(n+1, 3);
% ManipW = zeros(n+1, 1);
% X = zeros(n+1, 3);
% DetJ = zeros(n+1,1);

% t(1) = 0;
% Theta(1,1) = TH1;
% Theta(1,2) = TH2;
% Theta(1,3) = TH3;
% Theta(1,4) = TH4;
% Theta(1,5) = TH5;
% Theta(1,6) = TH6;
% Theta(1,7) = TH7;
% X(1,1) = T0_8(1,4);
% X(1,2) = T0_8(2,4);
% X(1,3) = T0_8(3,4);



for i=1:n
  
    TH1 = TH1-((2*pi)*floor(abs((TH1/(2*pi))))*sign(TH1));
    TH2 = TH2-((2*pi)*floor(abs((TH2/(2*pi))))*sign(TH2));
    TH3 = TH3-((2*pi)*floor(abs((TH3/(2*pi))))*sign(TH3));
    TH4 = TH4-((2*pi)*floor(abs((TH4/(2*pi))))*sign(TH4));
    TH5 = TH5-((2*pi)*floor(abs((TH5/(2*pi))))*sign(TH5));
    TH6 = TH6-((2*pi)*floor(abs((TH6/(2*pi))))*sign(TH6));
    TH7 = TH7-((2*pi)*floor(abs((TH7/(2*pi))))*sign(TH7));
    
% Jacobian function - Unmodified for Variable DH

   [DetJ0,J0Inv]=JacobianHanson_03252019(TH1,TH2,TH3,TH4,TH5,TH6,TH7);

% Finding the difference between desired (traj) and current position

   dx = Tarm(1,4,i) - T0_8(1, 4);
   dy = Tarm(2,4,i) - T0_8(2, 4);
   dz = Tarm(3,4,i) - T0_8(3, 4);
   
% Finding the difference between desired (traj) and current orientation
  
   eo = 0.5*(cross(T0_8(1:3,1),Tarm(1:3,1,i)) + cross(T0_8(1:3,2),Tarm(1:3,2,i)) + cross(T0_8(1:3,3),Tarm(1:3,3,i)));
   
% Convert distances into cartesian velocities.

   V_cart = [dx/dt; dy/dt; dz/dt; eo(1)/dt; eo(2)/dt; eo(3)/dt];
   

% Convert cartesian velocities into joint velocities

   V_joint = double(J0Inv) * V_cart;
   

% Convert joint velocities into joint position differences

   dTH1 = V_joint(1)*dt;
   dTH2 = V_joint(2)*dt;
   dTH3 = V_joint(3)*dt;
   dTH4 = V_joint(4)*dt;
   dTH5 = V_joint(5)*dt;
   dTH6 = V_joint(6)*dt;
   dTH7 = V_joint(7)*dt;
   

% Update new joint angles.
   %goes into the dynamixel
   TH1 = TH1 + (dTH1);
   TH2 = TH2 + (dTH2);
   TH3 = TH3 + (dTH3);
   TH4 = TH4 + (dTH4);
   TH5 = TH5 + (dTH5);
   TH6 = TH6 + (dTH6);
   TH7 = TH7 + (dTH7);

% Update forward kinematics input in radians
   %reads in the position from dynamixel 
   [P0_1, P0_2, P0_3, P0_4, P0_5, P0_6, P0_7, P0_8, T0_1, T0_2, T0_3, T0_4, T0_5, T0_6, T0_7, T0_8, P1_2, P2_3, P3_4, P4_5, P5_6, P6_7, P7_8, T1_2, T2_3, T3_4, T4_5, T5_6, T6_7, T7_8]=ForwardKinematicsHanson_03252019(TH1,TH2,TH3,TH4,TH5,TH6,TH7);
   T0_8=double(T0_8);


% Keep track of vectors needed for plotting 

%    t(i+1) = (t(i) + dt);
%    Theta(i+1,1) = TH1;
%    Theta(i+1,2) = TH2;
%    Theta(i+1,3) = TH3;
%    Theta(i+1,4) = TH4;
%    Theta(i+1,5) = TH5;
%    Theta(i+1,6) = TH6;
%    Theta(i+1,7) = TH7;
%    ThetaDot(i,1) = V_joint(1);
%    ThetaDot(i,2) = V_joint(2);
%    ThetaDot(i,3) = V_joint(3);
%    ThetaDot(i,4) = V_joint(4);
%    ThetaDot(i,5) = V_joint(5);
%    ThetaDot(i,6) = V_joint(6);
%    ThetaDot(i,7) = V_joint(7);
%    X(i+1,1) = T0_8(1,4);
%    X(i+1,2) = T0_8(2,4);
%    X(i+1,3) = T0_8(3,4);
   
% Update position vectors for simulation

   XLink1=double([0 P0_1(1)]);
   YLink1=double([0 P0_1(2)]);
   ZLink1=double([0 P0_1(3)]);

   XLink2=double([P0_1(1) P0_2(1)]);
   YLink2=double([P0_1(2) P0_2(2)]);
   ZLink2=double([P0_1(3) P0_2(3)]);

   XLink3=double([P0_2(1) P0_3(1)]);
   YLink3=double([P0_2(2) P0_3(2)]);
   ZLink3=double([P0_2(3) P0_3(3)]);

   XLink4=double([P0_3(1) P0_4(1)]);
   YLink4=double([P0_3(2) P0_4(2)]);
   ZLink4=double([P0_3(3) P0_4(3)]);

   XLink5=double([P0_4(1) P0_5(1)]);
   YLink5=double([P0_4(2) P0_5(2)]);
   ZLink5=double([P0_4(3) P0_5(3)]);

   XLink6=double([P0_5(1) P0_6(1)]);
   YLink6=double([P0_5(2) P0_6(2)]);
   ZLink6=double([P0_5(3) P0_6(3)]);

   XLink7=double([P0_6(1) P0_7(1)]);
   YLink7=double([P0_6(2) P0_7(2)]);
   ZLink7=double([P0_6(3) P0_7(3)]);
   
   XLink8=double([P0_7(1) P0_8(1)]);
   YLink8=double([P0_7(2) P0_8(2)]);
   ZLink8=double([P0_7(3) P0_8(3)]);
   

% Updating Simulation

    %AnimationHanson_03252019(n,i,TH1,TH2,TH3,TH4,TH5,TH6,TH7);

   pause(.00001)

% % Switched Y & Z to Account for Modified View in Simulation
   set(RLink1, 'XData', XLink1, 'YData', ZLink1, 'ZData', YLink1);
   set(RLink2, 'XData', XLink2, 'YData', ZLink2, 'ZData', YLink2);
   set(RLink3, 'XData', XLink3, 'YData', ZLink3, 'ZData', YLink3);
   set(RLink4, 'XData', XLink4, 'YData', ZLink4, 'ZData', YLink4);
   set(RLink5, 'XData', XLink5, 'YData', ZLink5, 'ZData', YLink5);
   set(RLink6, 'XData', XLink6, 'YData', ZLink6, 'ZData', YLink6);
   set(RLink7, 'XData', XLink7, 'YData', ZLink7, 'ZData', YLink7);
   set(RLink8, 'XData', XLink8, 'YData', ZLink8, 'ZData', YLink8);
   
%  ManipW(i)=DetJ0;
  
end
% [J0,J0Inv]=JacobianHanson_03252019(TH1,TH2,TH3,TH4,TH5,TH6,TH7);
ThetaDot(i+1,1)=0;
ThetaDot(i+1,2)=0;
ThetaDot(i+1,3)=0;
ThetaDot(i+1,4)=0;
ThetaDot(i+1,5)=0;
ThetaDot(i+1,6)=0;
ThetaDot(i+1,7)=0;

% figure
% plot(t,Theta,'linewidth',1);
% xlabel('time')
% ylabel('Joint Angles (radians)')
% title('Time vs Joint Angles')
% legend('JointR 1','JointR 2','JointR 3','JointR 4','JointR 5','JointR 6','JointR 7','location','northeastoutside')
% 
% figure
% plot(t,ThetaDot,'linewidth',1);
% xlabel('time')
% ylabel('Joint Velocity')
% title('Time vs Joint Velocity')
% legend('Theta 1 dot','Theta 2 dot','Theta 3 dot','Theta 4 dot','Theta 5 dot','Theta 6 dot','Theta 7 dot','location','northeastoutside')
% 
% figure
% plot(t,ManipW,'linewidth',1);
% xlabel('time')
% ylabel('Manipulability')
% title('Time vs Manipulability')
% legend('Manipulability','location','northeastoutside')
