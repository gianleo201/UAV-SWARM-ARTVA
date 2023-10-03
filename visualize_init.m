%% PLOT FRAMES

% real transmitter positon
plot(transmitter_real_pos(1),transmitter_real_pos(2),'x','Color','black','LineWidth',2,'MarkerSize',15);

% estimated transmitter position
VIZ_t_p_h = plot(transmitter_pos_hat(1),transmitter_pos_hat(2),'x','Color','red','MarkerSize',8);

% UAVS trajectories
VIZ_trajs = cell(2*N,1);
for i=1:N
    temp = reshape(recievers_pos_ode_history(1:STEP,i,1:2),STEP,2);
    VIZ_trajs{2*i-1} = plot(temp(1:end,1),temp(1:end,2),'LineWidth',1.5,'Color',color_list(i));
    VIZ_trajs{2*i} = plot(temp(end,1),temp(end,2),'o','MarkerSize',3,'Linewidth',1.5,'Color',color_list(i));
end

% time label
VIZ_TIMER = text(-15,14.5,"Simulation time: "+string(t_simulation(STEP))+" s");

% estimated mission time
VIZ_END_MISSION_TIME = text(-15,13,"Estimated endtime: "+string(t_simulation(STEP)+t_f)+" s");

% observability index
VIZ_OI = text(-15,11.5,"Observability index: "+string(OI_VAL(end)));

% transmitter position estimate variation
VIZ_TEV = text(-15,10,"Transmitter estimate variation: "+string(TRANSMITTER_ESTIMATE_VARIATION(end)));

if RECORD_VIDEO
    % capture the current frame
    frame = getframe(gcf); 
    writeVideo(writerObj, frame);
end
