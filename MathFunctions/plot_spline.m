%% Plot a given spline
% PLOT_SPLINE(SPLINE, TOL)
% Fplots the given spline (SPLINE) with a tolerance of 
% TOL

function plot_spline(spline, tol)
npnt_a = 1/tol;
for i = 1:1:(spline.points - 1)
    
    
    hold on
    
     [x_rng, y_rng] = pointsOnClothoid(spline.x(i), spline.y(i), ...
        spline.theta(i), spline.k(i), spline.dk(i), spline.L(i), npnt_a);
    plot(x_rng, y_rng, 'LineWidth',3);
    set(gca,'FontSize',20)
end
end