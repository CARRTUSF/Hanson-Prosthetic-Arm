% Carlo Canezo
% DH Parameter Code Spring 2019

function [DH]=DHArmHanson_03252019(TH1,TH2,TH3,TH4,TH5,TH6,TH7)

% Dimensions in CM (alpha,a,d,theta)

% DH =      [0            0          7.63      TH1;
%            (90*pi/180)  0          0         TH2;
%            (-90*pi/180) 0          25.498    TH3;
%            (90*pi/180)  0.296092   0         TH4;
%            (-90*pi/180) 0.35       23.00216  TH5;
%            (90*pi/180)  0          0         TH6;
%            (-90*pi/180) 0.5        0         TH7;
%            0            5          0         0];
       
       % Dimensions in CM (alpha,a,d,theta) Angles in Radians

DH =      [0             0          7.63          90*(pi/180)+TH1;
           (90*pi/180)   0          0.521         90*(pi/180)+TH2;
           (90*pi/180)   0          -25.498       -90*(pi/180)+TH3;
           (-90*pi/180)  0.296092   0             TH4;
           (-90*pi/180)  0.35       23.00216103   -90*(pi/180)+TH5;
           (-90*pi/180)  0          0             -90*(pi/180)+TH6;
           (90*pi/180)   0.5        0             TH7;
           0             5          0             0];
       