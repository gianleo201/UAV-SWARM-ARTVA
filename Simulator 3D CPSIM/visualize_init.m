%% PLOT FRAMES

% real transmitter positon
plot(transmitter_real_pos(1),transmitter_real_pos(2),'x','Color','black','LineWidth',4,'MarkerSize',20);

% % distributed estimated transmitter position
% VIZ_t_p_h_d = cell(N,1);
% for i = 1 : N
%     VIZ_t_p_h_d{i} = plot(UAV_NET.UAVS{i}.transmitter_pos_hat(1),UAV_NET.UAVS{i}.transmitter_pos_hat(2),'x','Color',color_list(i),'LineWidth',1.5,'MarkerSize',8);
% end

% estimated transmitter position
VIZ_t_p_h = plot(transmitter_pos_hat(1),transmitter_pos_hat(2),'x','Color','red','LineWidth',2.5,'MarkerSize',12);

% UAVS trajectories
VIZ_trajs = cell(2*N,1);
VIZ_drone_circle_d_safe = cell(N,1);
% VIZ_drone_circle_sensing = cell(N,1);
for i=1:N
    temp = reshape(recievers_pos_ode_history(1:STEP,i,1:2),STEP,2);
    VIZ_trajs{2*i-1} = plot(temp(1:end,1),temp(1:end,2),'LineWidth',1.5,'Color',color_list(i));
    VIZ_trajs{2*i} = plot(temp(end,1),temp(end,2),'o','MarkerSize',3,'Linewidth',1.5,'Color',color_list(i)); 
    [cplt,~]= plotCircle(temp(end,1:2),d_safe/2,'--',1,color_list(i),false);
    VIZ_drone_circle_d_safe{i} = cplt;
%     [cplt,~]= plotCircle(temp(end,1:2),2*d_safe,':',0.5,color_list(i),false);
%     VIZ_drone_circle_sensing{i} = cplt;
end


axis_position = getpixelposition(gca);
f_width = axis_position(3);
f_height = axis_position(4);

% time label
VIZ_TIMER = text('Units','Pixels','Position',[5 f_height-5],'String',"Simulation time: "+string(t_simulation(STEP))+" s");

% estimated mission time
% VIZ_END_MISSION_TIME = text(-15,13,"Estimated endtime: "+string(t_simulation(STEP)+t_f)+" s");
VIZ_END_MISSION_TIME = text('Units','Pixels','Position',[5 f_height-20],'String',"Estimated endtime: "+string(t_simulation(STEP)+t_f)+" s");

% observability index
% VIZ_OI = text(-15,11.5,"Observability index: "+string(OI_VAL(end)));
VIZ_OI = text('Units','Pixels','Position',[5 f_height-35],'String',"Observability index: "+string(OI_VAL(end)));

% transmitter position estimate variation
% VIZ_TEV = text(-15,10,"Transmitter estimate variation: "+string(TRANSMITTER_ESTIMATE_VARIATION(end)));
VIZ_TEV = text('Units','Pixels','Position',[5 f_height-50],'String',"Transmitter estimate variation: "+string(TRANSMITTER_ESTIMATE_VARIATION(end)));

% estimation device position
if COMPUTING_DEVICE_DELAY
    VIZ_ESTIMATION_DEVICE = plot(estimation_device_pos(1),estimation_device_pos(2),"*",'MarkerSize',20,'LineWidth',2,'Color','blue');
    plot(estimation_device_pos(1),estimation_device_pos(2),".",'MarkerSize',40,'Color','blue');
    text(estimation_device_pos(1)+3,estimation_device_pos(2),"Estimation device");
end

if RECORD_VIDEO
    % capture the current frame
    axis_position = getframe(gcf); 
    writeVideo(writerObj, axis_position);
end
