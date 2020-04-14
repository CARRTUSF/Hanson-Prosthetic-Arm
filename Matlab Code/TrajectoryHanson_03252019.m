% Carlo Canezo
% Hansen Arm Trajectory Spring 2019

function [Tarm, traj]=TrajectoryHanson_03252019(Rotf,Roti,Pf,Pi,n)

% Current Trajectory is Linear
% Trajectory Matrix determination Positions

dx=(Pf(1)-Pi(1))/n;
dy=(Pf(2)-Pi(2))/n;
dz=(Pf(3)-Pi(3))/n;

% Single Angle Determination for Initial Rotation Matrix % Refer to Paul pg 30-32

R = Roti'*Rotf;

nx = R(1,1);
ny = R(2,1);
nz = R(3,1);
ox = R(1,2);
oy = R(2,2);
oz = R(3,2);
ax = R(1,3);
ay = R(2,3);
az = R(3,3);

% sa = Single Angle or Theta in book eqn 1.82

sa = atan2((sqrt(((oz - ay)^2))+((ax - nz)^2)+((ny - ox)^2)),(nx + oy + az - 1));

% Normalization if sa is small
if sa < 0.001
    kx = 1;
    ky = 0;
    kz = 0;
    
% when sa is below 90 deg eqn 1.83 - 1.85
elseif sa < (pi/2) - 0.001 %% double check if plus or minus
    kx = (oz - ay)/(2*sin(sa));
    ky = (ax - nz)/(2*sin(sa));
    kz = (ny - ox)/(2*sin(sa));
    
% when sa is above 90 deg eqn 1.92 - 1.94 
else
    kx = sign(oz - ay)*sqrt((nx - cos(sa))/(1 - cos(sa)));
    ky = sign(ax - nz)*sqrt((oy - cos(sa))/(1 - cos(sa)));
    kz = sign(ny - ox)*sqrt((az - cos(sa))/(1 - cos(sa)));
    % eqn 1.98 - 1.103
    if (kx > ky) && (kx > kz) % kx largest
        ky = (ny + ox)/(2*kx*(1-cos(sa)));
        kz = (ax + nz)/(2*kx*(1-cos(sa)));
    elseif (ky > kx) && (ky > kz) % ky largest
        kx = (ny + ox)/(2*ky*(1-cos(sa)));
        kz = (oz + ny)/(2*ky*(1-cos(sa)));
    else % kz largest
        kx = (ax + nz)/(2*kz*(1-cos(sa)));
        ky = (oz + ay)/(2*kz*(1-cos(sa)));
    end
end

dsa = (sa - 0)/(n-1); %%

traj(1,1) = Pi(1);
traj(1,2) = Pi(2);
traj(1,3) = Pi(3);
traj(1,4) = 0;
Tarm(:,:,1) = [Roti Pi; 0 0 0 1];

for j=2:n
  traj(j,1)=Pi(1)+(j)*dx;
  traj(j,2)=Pi(2)+(j)*dy;
  traj(j,3)=Pi(3)+(j)*dz;
  traj(j,4)=(j-1)*dsa;
  
  da = traj(j,4);
  
  % eqn 1.73
  dR = [(((kx*kx)*(1-cos(da)))+   (cos(da))) (((ky*kx)*(1-cos(da)))-(kz*sin(da))) (((kz*kx)*(1-cos(da)))+(ky*sin(da)));
        (((kx*ky)*(1-cos(da)))+(kz*sin(da))) (((ky*ky)*(1-cos(da)))+   (cos(da))) (((kz*ky)*(1-cos(da)))-(kx*sin(da)));
        (((kx*kz)*(1-cos(da)))-(ky*sin(da))) (((ky*kz)*(1-cos(da)))+(kx*sin(da))) (((kz*kz)*(1-cos(da)))+   (cos(da)))];
  
 Tarm(:,:,j) = [Roti*dR [traj((j),1);traj((j),2);traj((j),3)]; 0 0 0 1];
  
end