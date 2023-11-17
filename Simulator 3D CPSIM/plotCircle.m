function [matlab_plot_handle, circle_pts] = plotCircle(p_center,radius,LineStyle,Linewidth,Color,only_pts)
    t = 0:2*pi/1000:2*pi;
    Xs = zeros(1,length(t));
    Ys = zeros(1,length(t));
    for i = 1:length(t)
        Xs(i) = p_center(1) + radius*cos(t(i));
        Ys(i) = p_center(2) + radius*sin(t(i));
    end
    if ~only_pts
        matlab_plot_handle = plot(Xs,Ys,LineStyle,'LineWidth',Linewidth,'Color',Color);
    else
        matlab_plot_handle = 0;
    end
    circle_pts = [Xs.' Ys.'];
end