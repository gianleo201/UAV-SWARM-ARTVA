if ~exist('VIZ_planned_trajs','var')
    VIZ_planned_trajs = cell(N,1);
    % show computed trajs

    animation_figure = figure(1); axis equal; hold on;
    grid on;
    if ~exist("hl_length","var")
        xlim([-30 30]);
        ylim([-30 30]);
    else
        temp_var = hl_length/2 + 10;
        xlim([-temp_var temp_var]);
        ylim([-temp_var temp_var]);
    end

    if ~USE_NMPC
        for i=1:N
            temp = squeeze(UAV_trajs(i,:,:));
            VIZ_planned_trajs{i} = plot(temp(:,1),temp(:,2),'--','LineWidth',1.5,'Color',color_list(i));
        end
    end
else
    for i=1:N
        temp = squeeze(UAV_trajs(i,:,:));
        set(VIZ_planned_trajs{i},'XData',temp(:,1),'YData',temp(:,2));
    end
end
