%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This script is responsible to read the fitted RANSAC plane and   %%%%
% 1) transform or project it to XY plane                           %%%%
% 2) Place a 3D cube at the new coordinate system's origin         %%%%
% 3) Project back the cube to the original scene coordinate system %%%%
% 4) Save the 3D box coordinates in the scene coordinate system    %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Load the ABCD, plane Coefficients obtained by the RANSAC Algorigthm
load('RANSAC Outputs/planeFitCoeff.mat', 'coeffs');
a = coeffs(1);
b = coeffs(2);
c = coeffs(3);
d = coeffs(4);
figureCount = 1;
%% Load the INLIER points extracted by the RANSAC Algorithtm
load('RANSAC Outputs/planeFitPoints.mat', 'points');
domPoints = points;

%% Load the original 3D point cloud generated by COLMAP
load('COLMAP 3D Points/3DPointCloud.mat', 'points');
cloudPoints3D = points;
fig = figure(figureCount);
figureCount = figureCount+1;
hold on;
grid

plot3(cloudPoints3D(:,2),cloudPoints3D(:,3),cloudPoints3D(:,4),'.r');

%% Perform the tranformation to XY Plane 

[x_new,y_new,z_new,a_new,b_new,c_new,d_new,figureCount,rotMatrix,offgrid,centerx,centery] = transformCoordinates(domPoints(:,1),domPoints(:,2),domPoints(:,3),a,b,c,d,cloudPoints3D,figureCount);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Visualization of the transformed inlier plane on XY plane  %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
fig = figure(figureCount);
figureCount = figureCount+1;
hold on;
grid
daspect([1 1 1])
[currx,curry] = meshgrid(-4:0.1:4);
currz          = zeros(size(currx));
surf(currx,curry,currz,'EdgeColor','none');
plot3(domPoints(:,1),domPoints(:,2),domPoints(:,3),'.b',0,0,0,'*g');
plot3(x_new,y_new,z_new,'+g',0,0,0,'*g');

[actx,acty] = meshgrid(-4:0.3:4);
actz        = (-1/c)*(a*actx + b*acty + d);
% surf(actx,acty,actz,'EdgeColor','none');

%% Define the corners of our 3D box on the XY plane
p1 = [-0.5 -0.5 0];
p2 = [0.5 -0.5 0];
p3 = [0.5 0.5 0];
p4 = [-0.5 0.5 0];
p5 = [-0.5 -0.5 -1];
p6 = [0.5 -0.5 -1];
p7 = [0.5 0.5 -1];
p8 = [-0.5 0.5 -1];

upperPoints = [p5;p6;p7;p8];
lowerPoints = [p1;p2;p3;p4];

%% Draw a Wireframe for the 3D Cube 
% LOWER FACE
[p12_uv,p12_dist] = fitLine(p1,p2);
[p23_uv,p23_dist] = fitLine(p2,p3);
[p34_uv,p34_dist] = fitLine(p3,p4);
[p41_uv,p41_dist] = fitLine(p4,p1);

% UPPER FACE
[p56_uv,p56_dist] = fitLine(p5,p6);
[p67_uv,p67_dist] = fitLine(p6,p7);
[p78_uv,p78_dist] = fitLine(p7,p8);
[p85_uv,p85_dist] = fitLine(p8,p5);

% SIDE FACES
[p15_uv,p15_dist] = fitLine(p1,p5);
[p26_uv,p26_dist] = fitLine(p2,p6);
[p37_uv,p37_dist] = fitLine(p3,p7);
[p48_uv,p48_dist] = fitLine(p4,p8);

pts12 = linspace(0,p12_dist,100);
pts23 = linspace(0,p23_dist,100);
pts34 = linspace(0,p34_dist,100);
pts41 = linspace(0,p41_dist,100);
pts56 = linspace(0,p56_dist,100);
pts67 = linspace(0,p67_dist,100);
pts78 = linspace(0,p78_dist,100);
pts85 = linspace(0,p85_dist,100);
pts15 = linspace(0,p15_dist,100);
pts26 = linspace(0,p26_dist,100);
pts37 = linspace(0,p37_dist,100);
pts48 = linspace(0,p48_dist,100); 

for i=1:size(pts12,2)
    p12_points(i,:) = p1 + p12_uv*pts12(1,i);
    p23_points(i,:) = p2 + p23_uv*pts23(1,i);
    p34_points(i,:) = p3 + p34_uv*pts34(1,i);
    p41_points(i,:) = p4 + p41_uv*pts41(1,i);
    p56_points(i,:) = p5 + p56_uv*pts56(1,i);
    p67_points(i,:) = p6 + p67_uv*pts67(1,i);
    p78_points(i,:) = p7 + p78_uv*pts78(1,i);
    p85_points(i,:) = p8 + p85_uv*pts85(1,i);
    
    p15_points(i,:) = p1 + p15_uv*pts15(1,i);
    p26_points(i,:) = p2 + p26_uv*pts26(1,i);
    p37_points(i,:) = p3 + p37_uv*pts37(1,i);
    p48_points(i,:) = p4 + p48_uv*pts48(1,i);
end

%% Transform the wireframe back to the scene coordinate system 
% Pc   = R(t-Pw)
% we need Pw given Pc (our coordinate system)

Plower = [p12_points;p23_points;p34_points;p41_points];
Pupper = [p56_points;p67_points;p78_points;p85_points];
Psider = [p15_points;p26_points;p37_points;p48_points];
P      = [Plower;Pupper;Psider];

temp        = (rotMatrix'*P')';
lowerPoints = (rotMatrix'*lowerPoints')';
upperPoints = (rotMatrix'*upperPoints')';

temp(:,3) = temp(:,3)+offgrid;
temp(:,1) = temp(:,1)+centerx;
temp(:,2) = temp(:,2)+centery;

lowerPoints(:,3) = lowerPoints(:,3)+offgrid;
lowerPoints(:,1) = lowerPoints(:,1)+centerx;
lowerPoints(:,2) = lowerPoints(:,2)+centery;

upperPoints(:,3) = upperPoints(:,3)+offgrid;
upperPoints(:,1) = upperPoints(:,1)+centerx;
upperPoints(:,2) = upperPoints(:,2)+centery;

wireframe = temp;

p1 = lowerPoints(1,:);
p2 = lowerPoints(2,:);
p3 = lowerPoints(3,:);
p4 = lowerPoints(4,:);
p5 = upperPoints(1,:);
p6 = upperPoints(2,:);
p7 = upperPoints(3,:);
p8 = upperPoints(4,:); 

poly_rectangle(p3, p4, p8, p7)
poly_rectangle(p1, p4, p8, p5)
poly_rectangle(p1, p2, p3, p4)
poly_rectangle(p2, p6, p7, p3)
poly_rectangle(p5, p6, p7, p8)
poly_rectangle(p1, p5, p8, p4)
hold off;

%% Finally store all the new 3D box scene coordinates
output3DboxPointsDir = '3D Box'; 
if ~exist(output3DboxPointsDir, 'dir')
    mkdir(output3DboxPointsDir);
end
save('3D Box/sceneBox3D.mat','wireframe','lowerPoints','upperPoints');