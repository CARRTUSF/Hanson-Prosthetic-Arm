% Carlo Canezo
% Animation Spring 2019

function AnimationHanson_03252019(n,i,TH1,TH2,TH3,TH4,TH5,TH6,TH7)

% Global Variables
global Animate



if i==1
    Animate = vrworld('\Carlo.wrl');
    open(Animate);

    Animate.f12.rotation=[0 0 1 TH1];
    Animate.f23.rotation=[1 0 0 TH2];
    Animate.f34.rotation=[0 1 0 TH3];
    Animate.f45.rotation=[0 0 1 TH4];
    Animate.f56.rotation=[0 1 0 TH5];
    Animate.f67.rotation=[1 0 0 TH6];
    Animate.f78.rotation=[0 0 1 TH7];

    view(Animate);
    
elseif i==(n)
    close(Animate);
    
else
    
    Animate.f12.rotation=[0 0 1 TH1];
    Animate.f23.rotation=[1 0 0 TH2];
    Animate.f34.rotation=[0 1 0 TH3];
    Animate.f45.rotation=[0 0 1 TH4];
    Animate.f56.rotation=[0 1 0 TH5];
    Animate.f67.rotation=[1 0 0 TH6];
    Animate.f78.rotation=[0 0 1 TH7];

    
end


