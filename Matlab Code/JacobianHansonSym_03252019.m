% Carlo Canezo
% Hansen Arm Jacobian Calculation Spring 2019

function [J0Invsym]=JacobianHansonSym_03252019
syms TH1 TH2 TH3 TH4 TH5 TH6 TH7 real

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

% Weighted Matrix Add In

W=[1 0 0 0 0 0 0;
0 2 0 0 0 0 0;
0 0 1 0 0 0 0;
0 0 0 1 0 0 0;
0 0 0 0 1 0 0;
0 0 0 0 0 1 0;
0 0 0 0 0 0 1];

InW=inv(W);
N=[theta1;theta2;theta3;theta4;theta5;theta6;theta7];
J8=jacobian([v88;w88],N);
J8Linear=jacobian(v88,N);
J8Angular=jacobian(w88,N);
R0_8=T0_8(1:3,1:3);
P0_8=T0_8(1:3,4);
J0Linear=(R0_8)*J8Linear;
J0Angular=(R0_8)*J8Angular;
J0=[J0Linear;J0Angular]; % Jacobian reference to 0
%DetJ0=det(J0); % Determinant of J0
%DetJ0=det(inv(W)*transpose(J0)*inv(J0*inv(W)*transpose(J0)));
J0Invsym=(inv(W))*transpose(J0)*(inv(J0*inv(W)*transpose(J0))); % Inverse for Qstep