
USE_ODE_MODEL = false;
PLOTTING = true;

%% connect to coppelia sim

if ~USE_ODE_MODEL
    client = RemoteAPIClient();
    sim = client.getObject('sim');
    UAV = sim.getObject('/Quadcopter');
    UAV_propeller = cell(1,4);
    for i=1:4
        UAV_propeller{i} = sim.getObject("/Quadcopter/propeller["+num2str(i-1)+"]/joint");
    end
    UAV_respondable = cell(1,4);
    for i=1:4
        UAV_respondable{i} = sim.getObject("/Quadcopter/propeller["+num2str(i-1)+"]/respondable");
    end
    TARGET = sim.getObject("/Quadcopter[0]/target");
    sim.setObjectPosition(UAV,sim.handle_world,{0,0,1});
end

% %% compute total inertia of quadrotor
% main_body = sim.getShapeInertia(UAV);
% main_body = cell2mat(main_body);
% main_body = reshape(main_body,3,3);
% total_inertia = main_body;
% % add inertia of the other bodies
% for i = 1:4
%     resp_UAV = sim.getObjectMatrix(UAV_respondable{i},UAV);
%     I = sim.getShapeInertia(UAV_respondable{i});
%     I = cell2mat(I);
%     I = reshape(I,3,3);
%     m = sim.getShapeMass(UAV_respondable{i});
%     resp_UAV = cell2mat(resp_UAV);
%     R0_i = [resp_UAV(1:3);resp_UAV(5:7);resp_UAV(9:11)];
%     p0_resp = [resp_UAV(4);resp_UAV(8);resp_UAV(12)];
%     I = R0_i * I * R0_i.';
%     total_inertia = total_inertia + I + m*((p0_resp.' * p0_resp)*eye(3) - p0_resp * p0_resp.');
% end
% %%

%% INIT

close all;

ref = [0 0 2 0];
onlineData.ref = ref;

U = [1.2753 1.2753 1.2753 1.2753];

x_k = [0 0 1 0 0 0 0 0 0 0 0 0];

X_history = [x_k];
U_history = [U];
CPU_TIME = [0];

%% INITIALIZE FL controller

FL_THRUST = sum(U);
FL_DTHRUST = 0;
MODEL_PARAMS_INIT;

%% START SIM

timevec = 0:0.05:50;
STEP = 1;

if ~USE_ODE_MODEL
    % enable the stepping mode on the client:
    client.setStepping(true);
    sim.startSimulation();
end

if PLOTTING
% plot position
fig1 = figure(1);
subplot(3,1,1); grid on; hold on;
title("X position");
xlabel("t [s]");
ylabel("position [m]");
xlim auto;
ylim([0 1.5]);
VIZ_positions_x = plot(timevec(1:STEP),X_history(1:STEP,1),'LineWidth',2.0);
hold off;
subplot(3,1,2); grid on; hold on;
title("Y position");
xlabel("t [s]");
ylabel("position [m]");
xlim auto;
ylim([0 1.5]);
VIZ_positions_y = plot(timevec(1:STEP),X_history(1:STEP,2),'LineWidth',2.0);
hold off;
subplot(3,1,3); grid on; hold on;
title("Z position");
xlabel("t [s]");
ylabel("position [m]");
xlim auto;
ylim([0 2.5]);
VIZ_positions_z = plot(timevec(1:STEP),X_history(1:STEP,3),'LineWidth',2.0);
hold off;

% plot thrust evolution
fig2 = figure(2);
subplot(4,1,1); grid on; hold on;
title("Thrust 1");
xlabel("t [s]");
ylabel("Force [N]");
xlim auto;
ylim([0 10]);
VIZ_thrust_1 = stairs(timevec(1:STEP),U_history(1:STEP,1),'LineWidth',2.0);
hold off;
subplot(4,1,2); grid on; hold on;
title("Thrust 2");
xlabel("t [s]");
ylabel("Force [N]");
xlim auto;
ylim([0 10]);
VIZ_thrust_2 = stairs(timevec(1:STEP),U_history(1:STEP,2),'LineWidth',2.0);
hold off;
subplot(4,1,3); grid on; hold on;
title("Thrust 3");
xlabel("t [s]");
ylabel("Force [N]");
xlim auto;
ylim([0 10]);
VIZ_thrust_3 = stairs(timevec(1:STEP),U_history(1:STEP,3),'LineWidth',2.0);
hold off;
subplot(4,1,4); grid on; hold on;
title("Thrust 4");
xlabel("t [s]");
ylabel("Force [N]");
xlim auto;
ylim([0 10]);
VIZ_thrust_4 = stairs(timevec(1:STEP),U_history(1:STEP,1),'LineWidth',2.0);
hold off;

% % CPU TIME 
% fig3 = figure(3); grid on; hold on; axis tight;
% title("CPU TIME");
% xlim auto;
% ylim([0 0.1]);
% xlabel("t [s]");
% ylabel("CPU TIME [s]");
% VIZ_CPU_TIME = plot(timevec(1:STEP),CPU_TIME(1:STEP),'LineWidth',2.0);
% hold off;

% angle evolution
fig4 = figure(4);
subplot(3,1,1); grid on; hold on;
title("roll angle");
xlabel("t [s]");
ylabel("angle [rad]");
xlim auto;
ylim([-pi/2 pi/2]);
VIZ_angles_roll = plot(timevec(1:STEP),X_history(1:STEP,10),'LineWidth',2.0);
hold off;
subplot(3,1,2); grid on; hold on;
title("pitch angle");
xlabel("t [s]");
ylabel("angle [rad]");
xlim auto;
ylim([-pi/2 pi/2]);
VIZ_angles_pitch = plot(timevec(1:STEP),X_history(1:STEP,11),'LineWidth',2.0);
hold off;
subplot(3,1,3); grid on; hold on;
title("yaw angle");
xlabel("t [s]");
ylabel("angle [rad]");
xlim auto;
ylim([-pi/2 pi/2]);
VIZ_angles_yaw = plot(timevec(1:STEP),X_history(1:STEP,12),'LineWidth',2.0);
hold off;
end


while timevec(STEP) < timevec(end)

    if ~USE_ODE_MODEL
        % extract UAV information
        x_k = getState(sim,UAV);
        curr_ref = sim.getObjectPosition(TARGET,sim.handle_world);
        ref_k = zeros(1,4);
        ref_k(4) = 0;
        for i = 1:3; ref_k(i) = double(curr_ref{i}); end
        onlineData.ref = ref_k;
%         x_k = [0 0 1 0 0 0 0 0 0 0 0 0];
%         DELTA_state = onlineData.X0(1,:) - x_k;
%     %     onlineData.md = [DELTA_state(1:3) DELTA_state(end)];
%         onlineData.md = DELTA_state;
    end

    % do nmpc step
    tic;
    [nmpc_mv, new_onlineData, info] = NMPC_STEP(Drone_NMPC, ...
                                            x_k.', ...
                                            U.', ...
                                            onlineData, ...
                                            true);
    time_taken = toc;
    CPU_TIME = [CPU_TIME time_taken];
    if info.ExitFlag <= 0 % check solution infeseability
        fprintf("no_feasable_solution_found\n");
    end
    U = nmpc_mv.';
    onlineData = new_onlineData;

%     % do FL step
%     x_k_aug = [x_k FL_THRUST FL_DTHRUST].';
%     T_mod = FL_step(x_k_aug,ref_k,model_params);
%     % compute real input
%     U = input_mapping([FL_THRUST;T_mod(2:4,1)],model_params).';
%     INV_MU = inv_input_mapping_matrix(model_params);
%     MAX_LIM = INV_MU*10*ones(4,1);
%     MIN_LIM = zeros(4,1);
%     % apply saturation function
%     for i = 1:length(1); if U(i)< 0; U(i)=0; else; if U(i) > 10; U(i) = 10; end; end; end
%     % update controller state
%     FL_THRUST = FL_THRUST + FL_DTHRUST*0.05;
%     FL_DTHRUST = FL_DTHRUST + T_mod(1)*0.05;
    

    if ~USE_ODE_MODEL
        % actuate uav
        control_UAV(sim,UAV_propeller,U);
        client.step();
    else
        % actuate model (by model integration)
        res = ode45(simulate_UAV(U.',CHECK_PARAMS),[0 0.05],x_k.');
        pause(0.05);
        x_k = res.y(:,end).';
    end
    
    % next step
    STEP = STEP + 1;
    X_history = [X_history;x_k];
    U_history = [U_history;U];

    if PLOTTING
    set(VIZ_positions_x,'XData',timevec(1:STEP),'YData',X_history(1:STEP,1));
    set(VIZ_positions_y,'XData',timevec(1:STEP),'YData',X_history(1:STEP,2));
    set(VIZ_positions_z,'XData',timevec(1:STEP),'YData',X_history(1:STEP,3));

    set(VIZ_angles_roll,'XData',timevec(1:STEP),'YData',X_history(1:STEP,10));
    set(VIZ_angles_pitch,'XData',timevec(1:STEP),'YData',X_history(1:STEP,11));
    set(VIZ_angles_yaw,'XData',timevec(1:STEP),'YData',X_history(1:STEP,12));

    set(VIZ_thrust_1,'XData',timevec(1:STEP),'YData',U_history(1:STEP,1));
    set(VIZ_thrust_2,'XData',timevec(1:STEP),'YData',U_history(1:STEP,2));
    set(VIZ_thrust_3,'XData',timevec(1:STEP),'YData',U_history(1:STEP,3));
    set(VIZ_thrust_4,'XData',timevec(1:STEP),'YData',U_history(1:STEP,4));
    
%     set(VIZ_CPU_TIME,'XData',timevec(1:STEP),'YData',CPU_TIME(1:STEP));
    drawnow;
    end

end

if ~USE_ODE_MODEL
    sim.stopSimulation();
end


%% functions

function UAV_state = getState(sim,UAV)
    T_temp = sim.getObjectMatrix(UAV,sim.handle_world);
    x = zeros(1,3);
    x(1) = double(T_temp{4});
    x(2) = double(T_temp{8});
    x(3) = double(T_temp{12});
    R = zeros(3,3);
    R(1,1) = double(T_temp{1}); 
    R(1,2) = double(T_temp{2}); 
    R(1,3) = double(T_temp{3});
    R(2,1) = double(T_temp{5});
    R(2,2) = double(T_temp{6});
    R(2,3) = double(T_temp{7});
    R(3,1) = double(T_temp{9});
    R(3,2) = double(T_temp{10});
    R(3,3) = double(T_temp{11});
    
%     rpy = rotm2eul(R,'ZYX');
%     rpy = rpy(end:-1:1);

    abg_cpsim = sim.getObjectOrientation(UAV,sim.handle_world);
    [yaw,pitch,roll] = sim.alphaBetaGammaToYawPitchRoll(abg_cpsim{1},abg_cpsim{2},abg_cpsim{3});
    rpy = [roll pitch yaw];

    [dx,w] = sim.getObjectVelocity(UAV+sim.handleflag_axis,sim.handle_world);
%     [dx_no_flag,w_no_flag] = sim.getObjectVelocity(UAV,sim.handle_world);
    dx_mat = zeros(1,3);
    w_mat = zeros(1,3);
    for i = 1:3
        dx_mat(i) = double(dx{i});
        w_mat(i) = double(w{i});
    end
    w_mat = (R.' * w_mat.').';
    UAV_state = [x dx_mat w_mat rpy];
end

function control_UAV(sim,UAV_propeller_list,controls)
    for i = 1:length(UAV_propeller_list)
        sim.setJointTargetVelocity(UAV_propeller_list{i},controls(i));
    end
end

function UAV_handle = simulate_UAV(u,params)
    UAV_handle =  @(t,x) quadrotor_model(x,u,params{:});
end
