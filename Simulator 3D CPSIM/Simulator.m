
clear;
EnvINIT; % initialize environment
MagneticFieldSensors; % build Symbolically H matrix

%% MAINLOOP SETUP

% robots initial position setup
recievers_pos = reciever_INIT;
recievers_pos_ode = [reciever_INIT(:,1:3) zeros(N,9)];
recievers_pos_ode_history = zeros(END_TIME/TIME_STEP,N,12);
recievers_pos_ode_history(1,:,:) = recievers_pos_ode;

a = 1.2920;
b = 1.0275;
M_real = diag([b^2 a^2 a^2]);

% RLS algorithm setup
beta_ff = 0.99; % forgetly factor
% RLS algorithm initialization
% S_RLS = diag(repmat(1,1,10)); % initial covariance matrix
% S_RLS = diag(repmat(1000,1,10)); % initial covariance matrix
% S_RLS = diag([1 1 1 1 1 1 ...
%              (hl_length^2)/12 (hl_length^2)/12 1 ...
%              (hl_length^4)/9]); % initial covariance matrix
S_RLS = diag([10 10 10 10 10 10 ...
             (hl_length^2)/12 (hl_length^2)/12 10 ...
             (hl_length^4)/9]); % initial covariance matrix
% transmitter_pos_hat = [0 0 0]; % initial guess for the transmitter
last_replanning_transmitter_pos_hat = transmitter_pos_hat; % keep memory at each replanning
M_hat = [4     0    0;
         0     1    0;
         0     0    1];
% m_11 m_12 m_13 m_22 m_23 m_33 p_t_bar rho
X_hat = [M_hat(1,1) M_hat(1,2) M_hat(1,3) M_hat(2,2) M_hat(2,3) M_hat(3,3) ...
        (M_hat*transmitter_pos_hat.').' ...
         transmitter_pos_hat*M_hat*transmitter_pos_hat.'];


% set NLP
t_f = 30; % [s]  ( first estimated mission time )
d_t = 5; % [m]
% ObjectiveWeights = [1 0.1 10]; % weigths used for last experiments
ObjectiveWeights = [1 1e-06/N 1e-01/N];
% ObjectiveWeights = [1 1e-02 1e-01];
OF = buildObjectiveFunction(ObjectiveWeights,N,TIME_STEP,N_approx_bernstain);
problem.objective = OF;
problem.solver = 'fmincon';

% problem.options = optimoptions("fmincon", ...
%                 "Display","iter-detailed",...
%                 "OptimalityTolerance",1e-02);

problem.options = optimoptions("fmincon",...
                "Display","iter-detailed",...
                "Algorithm","interior-point",...
                "EnableFeasibilityMode",true,...
                "SubproblemAlgorithm","cg",...
                "OptimalityTolerance",1e-02);

if ~USE_NMPC
    if NLP_PLANNING
        NLPOnline; % solve NLP
    else
        if EXPLORATION_TYPE == "R"
            L_t = 0.75*hl_length; % for radial exploration
        elseif EXPLORATION_TYPE == "V" || EXPLORATION_TYPE == "H"
            L_t = hl_length; % for v-h exploration
        end
        v_cruise = 0.8*v_max;
        et = 2*( L_t /v_cruise); % cruise velocity at 50%
        trap_num_samples = fix( et / TIME_STEP ) + 1;
        UAV_trajs = zeros(N,trap_num_samples,4);
        for i=1:N
            if EXPLORATION_TYPE == "R"
            reciever_END = reciever_INIT(i,1:2) + L_t * [cos((i-0.5)*angle_sector) sin((i-0.5)*angle_sector)]; % for radial exploration
            elseif EXPLORATION_TYPE == "V"
                reciever_END = reciever_INIT(i,1:2) + hl_length * [0 1]; % for vertical exploration
            elseif EXPLORATION_TYPE == "H"
                reciever_END = reciever_INIT(i,1:2) + hl_length * [1 0]; % for horizontal exploration
            end
            [xtemp,dxtemp] = straightTrapTraj(reciever_INIT(i,1:2),reciever_END,v_cruise,et,TIME_STEP);
            UAV_trajs(i,:,:) = [xtemp dxtemp];
        end
    end
end

% event-triggered replanning thresholds
t_replanning = 10; % replanning timer [s]
RHO_THRESHOLD = 2;
SIGMA_TRESHOLD = 0.7;

% compute number of pairs of interdistances
NUM_DIST = N*(N-1)/2;
INTER_DISTANCES = [zeros(1,NUM_DIST)];
VELOCITIES = [zeros(1,N)];
k = 1;
for i = 1:N
    VELOCITIES(end,i) = norm(recievers_pos_ode(i,4:6));
    for j = i+1:N
        INTER_DISTANCES(end,k) = norm(recievers_pos(i,:)-recievers_pos(j,:));
        k = k + 1;
    end
end

% obsevability index
my_temp = H_function(recievers_pos);
my_temp = my_temp*my_temp.';
last_matrix_sum = my_temp;
my_temp = min_sv_O(last_matrix_sum,min([N,10]));
OI_VAL = [my_temp];

% estimate variation
TRANSMITTER_ESTIMATE_VARIATION = [0];
ESTIMATE_VARIATION_THRESHOLD = 1e-04;

% estimate error
TRANSMITTER_ESTIMATE_ERROR = [norm(transmitter_real_pos(1:2)-transmitter_pos_hat(1:2))];

% initialize async-system
if COMPUTING_DEVICE_DELAY
    NOMINAL_TRANSMISSION_DISTANCE = 50; % [m] is the distance between any UAV and the estimation device such that the nominal transmission time is the sample time
    medium_velocity = NOMINAL_TRANSMISSION_DISTANCE/TIME_STEP; % [m/s]
    uncertain_time_range = 50*TIME_STEP; % [s]
    estimation_device_pos = [0 100 0];
    AS = Dispatcher(N,TIME_STEP,recievers_pos_ode,NaN,NaN);
end

% initialize UAV-network system
% network_matrix = [1 1 0 0 0;
%                   1 1 1 0 0;
%                   0 1 1 1 0;
%                   0 0 1 1 1;
%                   0 0 0 1 1];
% network_matrix = ones(5,5);
network_matrix = [1 1 0 0;
                  1 1 1 0;
                  0 1 1 1;
                  0 0 1 1];
% network_matrix = [1 1;
%                   1 1];
% check matrix dimensions
if size(network_matrix,1) ~= N || size(network_matrix,2) ~= N
    fprintf("ERROR: network matrix has wrong dimensions\n");
    return;
end
UAV_NET = UAVNetwork(N, ...
                     TIME_STEP, ...
                     network_matrix, ...
                     recievers_pos_ode, ...
                     X_hat, ...
                     S_RLS, ...
@(x) Y_function(x,transmitter_real_pos.',[a b],[M_real(1,1:3) M_real(2,2:3) M_real(3,3)],0,0), ...
                     Drone_NMPC, ...
                     d_safe, ...
                     v_max, ...
                     "3D", ...
                     model_params);

% DRLS estimate variation from standard RLS
NETWORK_ESTIMATES = UAV_NET.pull_estimates();
NETWORK_ESTIMATES_DEV = zeros(1,N);
for i = 1 : N
    NETWORK_ESTIMATES_DEV(i) = norm(NETWORK_ESTIMATES(i,1:3)-transmitter_pos_hat);
end
TRANSMITTER_ESTIMATE_DRLS_DEVIATION = [NETWORK_ESTIMATES_DEV];

%% CONNECT TO COPPELIA SIM

% connection to env
client = RemoteAPIClient();
sim = client.getObject('sim');

% get uav handles
UAV_propellers_list = cell(1,N);
UAV_handels = cell(N,2);
transmitter_handle = sim.getObject("/Transmitter");
for i =1:N
    UAV_handels{i,1} = sim.getObject("/Quadcopter["+num2str(i-1)+"]");
    UAV_handels{i,2} = sim.getObject("/Quadcopter["+num2str(i-1)+"]/target");
    temp_vec = zeros(1,4);
    for j=1:4
        temp_vec(j) = sim.getObject("/Quadcopter["+num2str(i-1)+"]/propeller["+num2str(j-1)+"]/joint");
    end
    UAV_propellers_list{i} = temp_vec;
end

% place UAVs properly
for i=1:N
    temp_cell = cell(1,3);
    for j=1:3; temp_cell{j} = reciever_INIT(i,j); end
    sim.setObjectPosition(UAV_handels{i,1},sim.handle_world,temp_cell);
end

% place the transmitter
temp_cell = cell(1,3);
for i = 1:3; temp_cell{i} = transmitter_real_pos(i); end
sim.setObjectPosition(transmitter_handle,sim.handle_world,temp_cell);

% targets in position estimate
for i = 1:N
    temp_cell = cell(1,3);
    for j = 1:3; temp_cell{j} = UAV_NET.UAVS{i}.X_ref(j); end
    sim.setObjectPosition(UAV_handels{i,2},sim.handle_world,temp_cell);
end

client.setStepping(true);

sim.startSimulation();

%% MAINLOOP EXECUTION

STEP = 1;
K_STEP = 2;

while t_simulation(STEP) < t_simulation(end)

    if ~USE_NMPC
        if ~COMPUTING_DEVICE_DELAY ||  t_simulation(STEP) >= 5
            % ending condition on estimate variation
            if STEP > 1 && TRANSMITTER_ESTIMATE_VARIATION(end) < ESTIMATE_VARIATION_THRESHOLD
                break;
            end
        end
    else
        % decentralized ending condition
        END_MISSION = UAV_NET.refresh_online_data();
        if END_MISSION && t_simulation(STEP) >= 5
            break;
        end
    end

    % augment simulation-data list
    INTER_DISTANCES = [INTER_DISTANCES;zeros(1,NUM_DIST)];
    VELOCITIES = [VELOCITIES;zeros(1,N)];

    % event-triggered replanning
    if  (NLP_PLANNING || DOUBLE_PHASE) && t_replanning <= 0
        ESTIMATE_ERROR = norm(transmitter_pos_hat-last_replanning_transmitter_pos_hat);
        OPI = OI_function(recievers_pos_ode_history(1:STEP,:,1:2));
        if  ESTIMATE_ERROR > RHO_THRESHOLD || ...
            OPI <= SIGMA_TRESHOLD
            last_replanning_transmitter_pos_hat = transmitter_pos_hat;
            K_STEP = 2; % reset steps
            NLPOnline; % solve again NLP
%             % show computed trajs
%             plotComputedTrajs;
%             % plot new estimated mission endtime
%             set(VIZ_END_MISSION_TIME,"String","Estimated endtime: "+string(t_simulation(STEP)+t_f)+" s");
        end
        t_replanning = t_f-TIME_STEP; % reset timer
    
    end
    
    % async case preliminarities
    if COMPUTING_DEVICE_DELAY
        % compute centralized async system step
        available_pos_ode = extractPosOde(AS.pull_latest_info((STEP-1)*TIME_STEP));
        available_pos = available_pos_ode(:,1:3);

        % compute new values ( with async sys measurment ) centralized
        H_num = H_function(available_pos);
        Y_num = Y_function(available_pos,transmitter_real_pos.',[a b],[M_real(1,1:3) M_real(2,2:3) M_real(3,3)],0,0);

        % compute new values decentralized
        UAV_NET.DRLS(beta_ff,Y_num,H_num,true,(STEP-1)*TIME_STEP);
    else 
        % compute new values centralized
        H_num = H_function(recievers_pos);
        Y_num = Y_function(recievers_pos,transmitter_real_pos.',[a b],[M_real(1,1:3) M_real(2,2:3) M_real(3,3)],0,0);

        % compute new values decentralized
        UAV_NET.DRLS(beta_ff,Y_num,H_num,false);
    end

    % RLS algorithm step
    S_RLS = inv(beta_ff * inv(S_RLS) + H_num*H_num.');
    X_hat = ( X_hat.' +  S_RLS*H_num*(Y_num-H_num.'*X_hat.') ).'; % new estimate of parameters vector;

    % extract transmitter position estimate
    M_hat = [X_hat(1) X_hat(2) X_hat(3);X_hat(2) X_hat(4) X_hat(5); X_hat(3) X_hat(5) X_hat(6)];
    old_transmitter_pos_hat = transmitter_pos_hat;
    transmitter_pos_hat = (inv(M_hat)*X_hat(7:9).').'; % new transmitter estimate


    if ~USE_NMPC
        UAV_NET.refresh_nmpc_state(recievers_pos_ode,false);
        % set current reference point for UAVS
%         reshape(squeeze(UAV_trajs(:,K_STEP,:)).',1,[]).'
%         repmat([transmitter_pos_hat(1:2) 0 0 0],1,N).'
        temp_dim = size(UAV_trajs);
        if K_STEP > temp_dim(2) % if trajs have ended stay at the end
            curr_traj = squeeze(UAV_trajs(:,end,:));
            curr_traj(:,3:4) = zeros(N,2);
%             UAV_references = reshape(curr_traj.',1,[]).';
            UAV_references = curr_traj;
        else
%             UAV_references = reshape(squeeze(UAV_trajs(:,K_STEP,:)).',1,[]).';
            UAV_references = squeeze(UAV_trajs(:,K_STEP,:));
        end
%         temp_refs = [1.25 0 0 0;3.75 0 0 0;6.25 0 0 0;8.75 0 0 0];
        U_FL = UAV_NET.FL_UAV_TEAM_step(UAV_references);
        for i = 1:N
            control_UAV(sim,UAV_propellers_list{i},U_FL(i,:));
        end
    else
        U_NMPC = UAV_NET.NMPC_UAV_TEAM_step();
        for i = 1:N
            control_UAV(sim,UAV_propellers_list{i},U_NMPC(i,:));
        end
    end

    % coppelia sim integration step
    client.step();
    for i = 1:N
        % get new recievers pos_ode
        recievers_pos_ode(i,:) = getState(sim,UAV_handels{i,1});
        % targets in position estimate
        temp_cell = cell(1,3);
        for j = 1:3; temp_cell{j} = UAV_NET.UAVS{i}.X_ref(j); end
        sim.setObjectPosition(UAV_handels{i,2},sim.handle_world,temp_cell);
    end
    recievers_pos = recievers_pos_ode(:,1:3);

    % next step
    STEP = STEP + 1;
    K_STEP = K_STEP + 1;
    t_replanning = t_replanning-TIME_STEP;

    if COMPUTING_DEVICE_DELAY
        % update centralized async system
        new_info = cell(1,N);
        for i=1:N
            random_alpha = rand(1,1);
%             delta_spawn_time = ( norm(recievers_pos(i,1:2) - estimation_device_pos(1:2))/medium_velocity ) + (2*random_alpha-1) * uncertain_time_range;
            delta_spawn_time = ( norm(recievers_pos(i,1:3) - estimation_device_pos(1:3))/medium_velocity ) + random_alpha * uncertain_time_range;
            spawn_time = (STEP-1)*TIME_STEP + delta_spawn_time;
            info_struct = {};
            info_struct.pos_ode = recievers_pos_ode(i,:);
            info_struct.spawn_time = spawn_time;
            new_info{i} = info_struct;
        end

        AS.push_info(new_info);
        % update decentrilized async system
        UAV_NET.info_transmission(recievers_pos_ode,(STEP-1)*TIME_STEP,medium_velocity,uncertain_time_range);
    end

    % compute velocities and interdistances
    k = 1;
    for i = 1:N
        VELOCITIES(end,i) = norm(recievers_pos_ode(i,4:6));
        for j = i+1:N
            INTER_DISTANCES(end,k) = norm(recievers_pos(i,:)-recievers_pos(j,:));
            k = k + 1;
        end
    end
  
    % update states history
    recievers_pos_ode_history(STEP,:,:) = recievers_pos_ode;

    % update Observability index value
    my_temp = H_function(recievers_pos);
    my_temp = my_temp*my_temp.';
    last_matrix_sum = last_matrix_sum + (my_temp - last_matrix_sum)/STEP;
    new_OI = min_sv_O(last_matrix_sum,min([N,10]));
    OI_VAL = [OI_VAL new_OI];

    % update estimate error and variation
    my2_temp = norm(transmitter_pos_hat-old_transmitter_pos_hat);
    TRANSMITTER_ESTIMATE_VARIATION = [TRANSMITTER_ESTIMATE_VARIATION my2_temp];
    TRANSMITTER_ESTIMATE_ERROR = [TRANSMITTER_ESTIMATE_ERROR norm(transmitter_real_pos(1:2)-transmitter_pos_hat(1:2))];

    % update DRLS estimate deviation form standard RLS
    NETWORK_ESTIMATES = UAV_NET.pull_estimates();
    NETWORK_ESTIMATES_DEV = zeros(1,N);
    for i = 1 : N
        NETWORK_ESTIMATES_DEV(i) = norm(NETWORK_ESTIMATES(i,1:2)-transmitter_pos_hat(1:2));
    end
    TRANSMITTER_ESTIMATE_DRLS_DEVIATION = [TRANSMITTER_ESTIMATE_DRLS_DEVIATION;NETWORK_ESTIMATES_DEV];

    if USE_NMPC
        if ~COMPUTING_DEVICE_DELAY
            % refresh online data for nmpc
            UAV_NET.refresh_nmpc_state(recievers_pos_ode,false);
        else
            UAV_NET.refresh_nmpc_state(recievers_pos_ode,true);
        end
    end

%     % visualize step
%     pause(TIME_STEP);
end

sim.stopSimulation();

%% PLOTS

if ~exist("tmp_dir","dir")
    mkdir . tmp_dir;
end

% plot inter distances
fig2 = figure(2);
grid on; hold on;
title("UAVs inter distances");
xlabel("time [s]");
ylabel("distance [m]");
xlim([0 TIME_STEP*STEP]);
for i = 1:NUM_DIST
    plot(t_simulation(1:STEP),INTER_DISTANCES(1:STEP,i),"LineWidth",2.0);
end
plot(t_simulation(1:STEP),repmat(d_safe,STEP,1),"--","Color","Black","LineWidth",1.5);
lgnd = cell(1,(N*(N-1)/2));
k = 1;
for i = 1 : N
    for j = i+1:N
        lgnd{k} = num2str(i)+"-"+num2str(j);
        k = k + 1;
    end
end
legend(lgnd{:});
hold off;
saveas(fig2,'./tmp_dir/inter_distances.png');
saveas(fig2,'./tmp_dir/inter_distances.fig');
saveas(fig2,'./tmp_dir/inter_distances','epsc');

% plot UAVs velocities
fig3 = figure(3);
grid on; hold on;
title("UAVs velocities");
xlabel("time [s]");
xlim([0 TIME_STEP*STEP]);
ylabel("velocity [m/s]");
for i = 1:N
    plot(t_simulation(1:STEP),VELOCITIES(1:STEP,i),"LineWidth",2.0,"Color",color_list(i));
end
plot(t_simulation(1:STEP),repmat(v_max,STEP,1),"--","Color","Black","LineWidth",1.5);
hold off;
saveas(fig3,'./tmp_dir/VELS.png');
saveas(fig3,'./tmp_dir/VELS.fig');
saveas(fig3,'./tmp_dir/VELS','epsc');

% plot observability index
fig4 = figure(4);
grid on; hold on;
title("Observability index");
xlabel("time [s]");
xlim([0 TIME_STEP*STEP]);
plot(t_simulation(1:STEP),OI_VAL,"Color","Black","LineWidth",1.5);
plot(t_simulation(1:STEP),repmat(SIGMA_TRESHOLD,STEP,1),"--","Color","Black","LineWidth",1.5);
hold off;
save SIGMA_TRESHOLD.mat SIGMA_TRESHOLD;
saveas(fig4,'./tmp_dir/OI.png');
saveas(fig4,'./tmp_dir/OI.fig');
saveas(fig4,'./tmp_dir/OI','epsc');

% plot transmitter position estimate variation
fig5 = figure(5);
grid on; hold on;
title("Transmitter position estimate variation")
xlabel("time [s]");
ylabel("[m]");
xlim([0 TIME_STEP*STEP]);
plot(t_simulation(1:STEP),TRANSMITTER_ESTIMATE_VARIATION,"Color","Black","LineWidth",1.5);
hold off;
saveas(fig5,'./tmp_dir/TEV.png');
saveas(fig5,'./tmp_dir/TEV.fig');
saveas(fig5,'./tmp_dir/TEV','epsc');

% plot transmitter position estimate error
fig6 = figure(6);
grid on; hold on;
title("Transmitter position estimate error");
xlabel("time [s]");
ylabel("[m]");
xlim([0 TIME_STEP*STEP]);
ylim([0 5]);
plot(t_simulation(1:STEP),TRANSMITTER_ESTIMATE_ERROR,"Color","Black","LineWidth",1.5);
hold off;
if COMPUTING_DEVICE_DELAY
    save TRANSMITTER_ESTIMATE_ERROR.mat TRANSMITTER_ESTIMATE_ERROR;
else
    save TRANSMITTER_ESTIMATE_ERROR_NO_DELAY.mat TRANSMITTER_ESTIMATE_ERROR;
end
saveas(fig6,'./tmp_dir/TEE.png');
saveas(fig6,'./tmp_dir/TEE.fig');
saveas(fig6,'./tmp_dir/TEE','epsc');

% plot DRLS deviation from standard RLS
fig7 = figure(7);
grid on; hold on;
title("DRLS deviation from centralized RLS");
xlabel("time [s]");
ylabel("[m]");
xlim([0 TIME_STEP*STEP]);
ylim([0 5]);
for i = 1 : N
    plot(t_simulation(1:STEP),TRANSMITTER_ESTIMATE_DRLS_DEVIATION(:,i).',"Color",color_list(i),"LineWidth",1.5);
end
hold off;
saveas(fig7,'./tmp_dir/DRLS_deviation.png');
saveas(fig7,'./tmp_dir/DRLS_deviation.fig');
saveas(fig7,'./tmp_dir/DRLS_deviation','epsc');

if USE_NMPC
    % plot cputime
    fig8 = figure(8); grid on; hold on;
    title("NMPC CPU TIMES");
    xlabel("time [s]");
    ylabel("CPU-TIME [s]");
    xlim([0 TIME_STEP*STEP]);
    ylim([0 2*TIME_STEP]);
    for i = 1 : N
        plot(t_simulation(1:STEP-1),UAV_NET.UAVS{i}.CPU_TIME(1:STEP-1),"Color",color_list(i),"LineWidth",1.5);
        
    end
    plot(t_simulation(1:STEP-1),repmat(TIME_STEP,STEP-1,1),"--","Color","black","LineWidth",1.5);
    lgnd = cell(1,N+1);
    for i = 1 :N
        lgnd{i} = "CPU TIME UAV "+num2str(i);
    end
    lgnd{i+1} = "MAX CPU TIME";
    legend(lgnd);
    hold off;
    saveas(fig8,'./tmp_dir/CPU_TIME.png');
    saveas(fig8,'./tmp_dir/CPU_TIME.fig');
    saveas(fig8,'./tmp_dir/CPU_TIME','epsc');
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
        sim.setJointTargetVelocity(UAV_propeller_list(i),controls(i));
    end
end