function [x_new,y_new,z_new,a1,b1,c1,d1,figCount,rotMatrix,offgrid,centerx,centery] = transformCoordinates(x,y,z,a,b,c,d,init,figureCount)

%   Plotting the origin of the camera coordinate system.
    plot3(0,0,0,'*g');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Show the distinction between 3d point cloud and inlier plane %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fig = figure(figureCount);
    figureCount = figureCount+1;
    hold on;
    grid
    
    [actx,acty]     = meshgrid(-10:0.3:10);
    actz            = (-1/c)*(a*actx + b*acty + d);
    surf(actx,acty,actz);
    plot3(x,y,z,'.b',0,0,0,'*g');
    plot3(init(:,2),init(:,3),init(:,4),'.r');
    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Translate the inliers plane to pass through origin %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    fig = figure(figureCount);
    figureCount = figureCount+1;
    hold on;
    grid
    
    plot3(x,y,z,'+b');
    offgrid = -d/c;
    z_new   = z - offgrid;
    x_new   = x;
    y_new   = y;
    [actx,acty] = meshgrid(-10:0.3:10);
    actz          = (-1/c)*(a*actx + b*acty);
    surf(actx,acty,actz);
    plot3(x_new,y_new,z_new,'+r',0,0,0,'og');
    hold off;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Translate the inliers plane to pass through center of inliers %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    minx = min(x);
    maxx = max(x);
    miny = min(y);
    maxy = max(y);
    centerx = minx + (maxx-minx)/2;
    centery = miny + (maxy-miny)/2; 
    x_new = x_new - centerx;
    y_new = y_new - centery;
    z_new = z_new;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find the angle and axis of rotation to project on x-y plane %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    fig = figure(figureCount);
    figureCount = figureCount+1;
    hold on;
    grid
   
    [actx,acty] = meshgrid(-10:0.3:10);
    actz          = (-1/c)*(a*actx + b*acty+d);
    surf(actx,acty,actz);
    plot3(x,y,z,'+b');
    
    p = a/sqrt((a*a + b*b + c*c));
    q = b/sqrt((a*a + b*b + c*c));
    r = c/sqrt((a*a + b*b + c*c));
    v = [-p;-q;-r];                 % Normal vector of the inlier plane
    k = [0;0;1];                    % Normal vector of the xy plane
    cosTheta  = dot(v,k)/norm(v);   % angle between inlier plane and xy plane
    sinTheta  = sqrt((1 - cosTheta*cosTheta));
       
    u   = cross(v,k);   % vector along the axis of rotation 
    u   = u/norm(u);    % unit vector along the direction of axis of rotation
    u1  = u(1);
    u2  = u(2);

    rotMatrix = [(cosTheta + u1*u1*(1-cosTheta)) (u1*u2*(1-cosTheta))            (u2*sinTheta);
                 (u1*u2*(1-cosTheta))            (cosTheta + u2*u2*(1-cosTheta)) (-1*u1*sinTheta);
                 (-1*u2*sinTheta)                (u1*sinTheta)                   (cosTheta)];   
   
    oldxyz  = [x_new';y_new';z_new'];   % current inlier points (not necessarily on the inlier plane).
    newxyz  = rotMatrix*oldxyz;         % Transformed the inlier points to xy-plane,
    x_new   = newxyz(1,:)';             % so, points may not be exactly on the xy-plane but close.
    y_new   = newxyz(2,:)';
    z_new   = newxyz(3,:)';

    [finalx,finaly] = meshgrid(-10:0.3:10);
    finalz          = zeros(size(finalx));
    surf(finalx,finaly,finalz);
    plot3(x_new,y_new,z_new,'+g',0,0,0,'*g');
    hold off;
    a1 = 0;
    b1 = 0;
    c1 = 1;
    d1 = 0;
    figCount = figureCount;
    
    
end