% Carlo Canezo
% Hansen Arm Joint Limits Spring 2019

function [MinJointAngles,MaxJointAngles] = JointLimitsHanson_03252019()

%Case 1
% Minimum SAngles [TH1;TH2;TH3;TH4;TH5;TH6;TH7] Converted to Radians

MinJointAngles = [-179;-140;-180;0.01;-180;-45;-30]*(pi/180);

% Maximum Angles [TH1;TH2;TH3;TH4;TH5;TH6;TH7] Converted to Radians

MaxJointAngles = [179;0.01;180;125;180;65;30]*(pi/180);

end

