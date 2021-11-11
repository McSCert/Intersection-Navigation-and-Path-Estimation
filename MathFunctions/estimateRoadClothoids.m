%% Estimate the a spline of clothoids for a given road 
% [EQ] = ESTIMATEROADCLOTHOIDS(ROADCENTERS, BANKANGLE,POINTS)
% Estimates a spline of clothoids based on the road center points
% (ROADCENTERS), the bankangle (BANKANGLE) and the number of points
% (POINTS)
%
% OUTPUT
%
% EQ - the spline of clothoids that models the road
% 


function [eq] = estimateRoadClothoids(roadCenters, bankangle,points)

% find the course angles at each waypoint
%eq.theta0 = matlabshared.tracking.internal.scenario.clothoidG2fitCourse(roadCenters);
%%matlabshared.tracking.internal.scenario.clothoidG1fit(hip(i),course(i),hip(i+1),course(i+1));

eq.x = roadCenters(:, 1);
eq.y = roadCenters(:, 2);
eq.points = points;

[eq.theta,eq.k,eq.dk,eq.L,eq.nevalG1, ...
    eq.nevalF,eq.iter,eq.Fvalue,eq.Fgradnorm] = G1spline(roadCenters(:,1:2)) ;


% report cumulative horizontal distance traveled from initial point.
hcd = [0; cumsum(eq.L)];

% get the piecewise polynomial for elevation based upon xy distance traveled.
z = roadCenters(:,3);
eq.vpp = fit(hcd, z, 'smoothingspline');

% do the same for the banking angles
eq.bpp = fit(hcd, bankangle, 'smoothingspline');