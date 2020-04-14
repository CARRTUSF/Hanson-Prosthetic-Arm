% Carlo Canezo
% Hansen Arm Jacobian Calculation with Optimization Spring 2019

function [DetJ0,J0Inv]=JacobianHanson_03252019(TH1,TH2,TH3,TH4,TH5,TH6,TH7)

global dHo;

% Transformation Matrices

[P0_1, P0_2, P0_3, P0_4, P0_5, P0_6, P0_7, P0_8, T0_1, T0_2, T0_3, T0_4, T0_5, T0_6, T0_7, T0_8, P1_2, P2_3, P3_4, P4_5, P5_6, P6_7, P7_8, T1_2, T2_3, T3_4, T4_5, T5_6, T6_7, T7_8]=ForwardKinematicsHanson_03252019(TH1,TH2,TH3,TH4,TH5,TH6,TH7);

% Jacobian Velocity Propagation

% Frame 0
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 % Theta dots
% Velocity Propagation

w00=[0;0;0];
v00=[0;0;0];

% Frame 1 revolute

w11=transpose(T0_1(1:3,1:3))*w00 + [0;0;theta1];
v11=transpose(T0_1(1:3,1:3))*(v00 + cross(w00,P0_1));

% Frame 2 revolute

w22=transpose(T1_2(1:3,1:3))*w11 + [0;0;theta2];
v22=transpose(T1_2(1:3,1:3))*(v11 + cross(w11,P1_2));

% Frame 3 revolute

w33=transpose(T2_3(1:3,1:3))*w22 + [0;0;theta3];
v33=transpose(T2_3(1:3,1:3))*(v22 + cross(w22,P2_3));

% Frame 4 revolute

w44=transpose(T3_4(1:3,1:3))*w33 + [0;0;theta4];
v44=transpose(T3_4(1:3,1:3))*(v33 + cross(w33,P3_4));

% Frame 5 revolute

w55=transpose(T4_5(1:3,1:3))*w44 + [0;0;theta5];
v55=transpose(T4_5(1:3,1:3))*(v44 + cross(w44,P4_5));

% Frame 6 revolute

w66=transpose(T5_6(1:3,1:3))*w55 + [0;0;theta6];
v66=transpose(T5_6(1:3,1:3))*(v55 + cross(w55,P5_6));

% Frame 7 revolute

w77=transpose(T6_7(1:3,1:3))*w66 + [0;0;theta7];
v77=transpose(T6_7(1:3,1:3))*(v66 + cross(w66,P6_7));

% Frame 8

w88=transpose(T7_8(1:3,1:3))*w77 + [0;0;0];
v88=transpose(T7_8(1:3,1:3))*(v77 + cross(w77,P7_8));


% Joint Limits Initiated

[MinJointAngles,MaxJointAngles] = JointLimitsHanson_03252019;

% Optimization Function Refer to Paper by Chan & Dubey
% Initializing dH/d(Thetai) Gradient

dH = [0;0;0;0;0;0;0];
CurrentAngles = [TH1;TH2;TH3;TH4;TH5;TH6;TH7];
% CurrentAnglesDeg = [TH1;TH2;TH3;TH4;TH5;TH6;TH7]*(180/pi);

for i=1:7 % # of Joints
    dH(i) = (((MaxJointAngles(i)-MinJointAngles(i))^2)*(2*CurrentAngles(i)-MaxJointAngles(i)-MinJointAngles(i)))/(4*((MaxJointAngles(i)-CurrentAngles(i))^2)*((CurrentAngles(i)-MinJointAngles(i))^2));

%     Forced Redefinitions
                if abs(dH(i)) < abs(dHo(i)) && CurrentAngles(i) < MaxJointAngles(i) && CurrentAngles(i) > MinJointAngles(i)
                dH(i)=0;
                
                elseif abs(dH(i)) < abs(dHo(i)) && (CurrentAngles(i) >= MaxJointAngles(i) || CurrentAngles(i) <= MinJointAngles(i))
%                 dH(i)=99999999999999999;
                dH(i)=double(inf);
                
                elseif abs(dH(i)) > abs(dHo(i)) && (CurrentAngles(i) >= MaxJointAngles(i) || CurrentAngles(i) <= MinJointAngles(i))
                dH(i)=0;
                end
end

dHo = dH;
 
% Weighted Matrix Add In Refer to Paper by Chan & Dubey

W=diag(1*[1;1;1;1;1;1;1]+1*abs(dH));
WdiagonalTerm=diag(W);

% W=diag([1;1;1;1;1;1;1]);

InW=diag([1/WdiagonalTerm(1); 1/WdiagonalTerm(2); 1/WdiagonalTerm(3); 1/WdiagonalTerm(4); 1/WdiagonalTerm(5); 1/WdiagonalTerm(6); 1/WdiagonalTerm(7)]);
% InW=inv(W);

N=[theta1;theta2;theta3;theta4;theta5;theta6;theta7];

% J8=jacobian([v88;w88],N);

J8Linear=double(jacobian(v88,N));

J8Angular=double(jacobian(w88,N));

R0_8=T0_8(1:3,1:3);

P0_8=T0_8(1:3,4);

J0Linear=(R0_8)*J8Linear;
J0Angular=(R0_8)*J8Angular;
J0=double([J0Linear;J0Angular]); % Jacobian reference to 0


DetJ0 = double(sqrt(det(J0*W*transpose(J0))));
w0 = 1000000;
k0 = 2000;

Calc = double(J0*InW*transpose(J0));

% Dynamic Scale Factor for Singularity Robustness Refer to Nakamura Pg 268
% Eqn 9.79

if DetJ0 < w0
    
  k = k0*((1-(DetJ0/w0))^2);
  
else
    k = 0;
end
J0Inv=double((InW)*transpose(J0))*(inv(Calc+(k)*eye(6))); % Pseudo Inverse Method with Singularity Robustness