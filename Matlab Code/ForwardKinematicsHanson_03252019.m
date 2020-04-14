% Carlo Canezo
% Hansen Arm Forward Kinematics Spring 2019
% Position Guessing to determine Acceptable Transformation Matrices 

function [P0_1, P0_2, P0_3, P0_4, P0_5, P0_6, P0_7, P0_8, T0_1, T0_2, T0_3, T0_4, T0_5, T0_6, T0_7, T0_8, P1_2, P2_3, P3_4, P4_5, P5_6, P6_7, P7_8, T1_2, T2_3, T3_4, T4_5, T5_6, T6_7, T7_8]=ForwardKinematicsHanson_03252019(TH1,TH2,TH3,TH4,TH5,TH6,TH7) % Angle Input in Radians

% Global DH
global DH
[DH]=DHArmHanson_03252019(TH1,TH2,TH3,TH4,TH5,TH6,TH7);

% Transformation Matrices

T0_1=[cos(DH(1,4)) -sin(DH(1,4)) 0 DH(1,2);
(sin(DH(1,4))*cos(DH(1,1))) (cos(DH(1,4))*cos(DH(1,1))) -sin(DH(1,1)) (-sin(DH(1,1))*(DH(1,3)));
(sin(DH(1,4))*sin(DH(1,1))) (cos(DH(1,4))*sin(DH(1,1))) cos(DH(1,1)) (cos(DH(1,1))*(DH(1,3)));
0 0 0 1];

T1_2=[cos(DH(2,4)) -sin(DH(2,4)) 0 DH(2,2);
(sin(DH(2,4))*cos(DH(2,1))) (cos(DH(2,4))*cos(DH(2,1))) -sin(DH(2,1)) (-sin(DH(2,1))*(DH(2,3)));
(sin(DH(2,4))*sin(DH(2,1))) (cos(DH(2,4))*sin(DH(2,1))) cos(DH(2,1)) (cos(DH(2,1))*(DH(2,3)));
0 0 0 1];

T2_3=[cos(DH(3,4)) -sin(DH(3,4)) 0 DH(3,2);
(sin(DH(3,4))*cos(DH(3,1))) (cos(DH(3,4))*cos(DH(3,1))) -sin(DH(3,1)) (-sin(DH(3,1))*(DH(3,3)));
(sin(DH(3,4))*sin(DH(3,1))) (cos(DH(3,4))*sin(DH(3,1))) cos(DH(3,1)) (cos(DH(3,1))*(DH(3,3)));
0 0 0 1];

T3_4=[cos(DH(4,4)) -sin(DH(4,4)) 0 DH(4,2);
(sin(DH(4,4))*cos(DH(4,1))) (cos(DH(4,4))*cos(DH(4,1))) -sin(DH(4,1)) (-sin(DH(4,1))*(DH(4,3)));
(sin(DH(4,4))*sin(DH(4,1))) (cos(DH(4,4))*sin(DH(4,1))) cos(DH(4,1)) (cos(DH(4,1))*(DH(4,3)));
0 0 0 1];

T4_5=[cos(DH(5,4)) -sin(DH(5,4)) 0 DH(5,2);
(sin(DH(5,4))*cos(DH(5,1))) (cos(DH(5,4))*cos(DH(5,1))) -sin(DH(5,1)) (-sin(DH(5,1))*(DH(5,3)));
(sin(DH(5,4))*sin(DH(5,1))) (cos(DH(5,4))*sin(DH(5,1))) cos(DH(5,1)) (cos(DH(5,1))*(DH(5,3)));
0 0 0 1];

T5_6=[cos(DH(6,4)) -sin(DH(6,4)) 0 DH(6,2);
(sin(DH(6,4))*cos(DH(6,1))) (cos(DH(6,4))*cos(DH(6,1))) -sin(DH(6,1)) (-sin(DH(6,1))*(DH(6,3)));
(sin(DH(6,4))*sin(DH(6,1))) (cos(DH(6,4))*sin(DH(6,1))) cos(DH(6,1)) (cos(DH(6,1))*(DH(6,3)));
0 0 0 1];

T6_7=[cos(DH(7,4)) -sin(DH(7,4)) 0 DH(7,2);
(sin(DH(7,4))*cos(DH(7,1))) (cos(DH(7,4))*cos(DH(7,1))) -sin(DH(7,1)) (-sin(DH(7,1))*(DH(7,3)));
(sin(DH(7,4))*sin(DH(7,1))) (cos(DH(7,4))*sin(DH(7,1))) cos(DH(7,1)) (cos(DH(7,1))*(DH(7,3)));
0 0 0 1];

T7_8=[cos(DH(8,4)) -sin(DH(8,4)) 0 DH(8,2);
(sin(DH(8,4))*cos(DH(8,1))) (cos(DH(8,4))*cos(DH(8,1))) -sin(DH(8,1)) (-sin(DH(8,1))*(DH(8,3)));
(sin(DH(8,4))*sin(DH(8,1))) (cos(DH(8,4))*sin(DH(8,1))) cos(DH(8,1)) (cos(DH(8,1))*(DH(8,3)));
0 0 0 1];

% Transformation Equation respect to Zero Frame

% T0_1=T0_1;
T0_2=T0_1*T1_2;
T0_3=T0_1*T1_2*T2_3;
T0_4=T0_1*T1_2*T2_3*T3_4;
T0_5=T0_1*T1_2*T2_3*T3_4*T4_5;
T0_6=T0_1*T1_2*T2_3*T3_4*T4_5*T5_6;
T0_7=T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_7;
T0_8=T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_7*T7_8;

% Position Vectors respect to each frames

% P0_1=T0_1(1:3,4);
P0_2=T0_2(1:3,4);
P0_3=T0_3(1:3,4);
P0_4=T0_4(1:3,4);
P0_5=T0_5(1:3,4);
P0_6=T0_6(1:3,4);
P0_7=T0_7(1:3,4);
P0_8=T0_8(1:3,4);

% Position Vectors respect to Zero

P0_1=T0_1(1:3,4);
P1_2=T1_2(1:3,4);
P2_3=T2_3(1:3,4);
P3_4=T3_4(1:3,4);
P4_5=T4_5(1:3,4);
P5_6=T5_6(1:3,4);
P6_7=T6_7(1:3,4);
P7_8=T7_8(1:3,4);