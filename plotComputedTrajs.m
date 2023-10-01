if ~exist('VIZ_planned_trajs','var')
    VIZ_planned_trajs = cell(N,1);
    % show computed trajs

    animation_figure = figure(1); hold on;
    grid on;
    xlim([-15 15]);
    ylim([-15 15]);
    
    for i=1:N
        temp = squeeze(UAV_trajs(i,:,:));
        VIZ_planned_trajs{i} = plot(temp(:,1),temp(:,2),'--','LineWidth',1.5,'Color',color_list(i));
    end
else
    for i=1:N
        temp = squeeze(UAV_trajs(i,:,:));
        set(VIZ_planned_trajs{i},'XData',temp(:,1),'YData',temp(:,2));
    end
end
