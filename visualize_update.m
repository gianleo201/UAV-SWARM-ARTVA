%% Update plot

% update estimated transmitter positon
set(VIZ_t_p_h,'XData',transmitter_pos_hat(1),'YData',transmitter_pos_hat(2));

% update distributed estimated transmitter position
for i = 1 : N
    set(VIZ_t_p_h_d{i},'XData',UAV_NET.UAVS{i}.transmitter_pos_hat(1),'YData',UAV_NET.UAVS{i}.transmitter_pos_hat(2));
end

% update UAVs trajectories plot
for i=1:N
    temp = reshape(recievers_pos_ode_history(1:STEP,i,1:2),STEP,2);
    VIZ_trajs{2*i-1}.XData = temp(1:end,1);
    VIZ_trajs{2*i-1}.YData = temp(1:end,2);
    VIZ_trajs{2*i}.XData = temp(end,1);
    VIZ_trajs{2*i}.YData = temp(end,2);
end

% update visual timer
set(VIZ_TIMER,"String","Simulation time: "+string(t_simulation(STEP))+" s");

% update observability index
set(VIZ_OI,"String","Observability index: "+string(OI_VAL(end)));

% update transmitter position estimate variation
set(VIZ_TEV,"String","Transmitter estimate variation: "+string(TRANSMITTER_ESTIMATE_VARIATION(end)));

% trigger update
drawnow;

if RECORD_VIDEO
    % capture the current frame
    frame = getframe(gcf); 
    writeVideo(writerObj, frame);
end
