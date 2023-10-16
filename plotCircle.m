function matlab_plot_handle = plotCircle(p_center,radius,LineStyle,Linewidth,Color)
    t = 0:2*pi/1000:2*pi;
    Xs = zeros(1,length(t));
    Ys = zeros(1,length(t));
    for i = 1:length(t)
        Xs(i) = p_center(1) + radius*cos(t(i));
        Ys(i) = p_center(2) + radius*sin(t(i));
    end
    matlab_plot_handle = plot(Xs,Ys,LineStyle,'LineWidth',Linewidth,'Color',Color);
end