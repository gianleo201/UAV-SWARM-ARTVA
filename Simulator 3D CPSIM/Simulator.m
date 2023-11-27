
clear;
EnvINIT; % initialize environment

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
S_RLS = diag([1 1 1 1 1 1 ...
             (hl_length^2)/12 (hl_length^2)/12 1 ...
             (hl_length^4)/9]); % initial covariance matrix
% S_RLS = diag([10 10 10 10 10 10 ...
%              (hl_length^2)/12 (hl_length^2)/12 1 ...
%              (hl_length^4)/9]); % initial covariance matrix
% transmitter_pos_hat = [0 0 0]; % initial guess for the transmitter
last_replanning_transmitter_pos_hat = transmitter_pos_hat; % keep memory at each replanning
M_hat = [b^2   0      0;
         0     a^2    0;
         0     0    a^2];
% m_11 m_12 m_13 m_22 m_23 m_33 p_t_bar rho
X_hat = [M_hat(1,1) M_hat(1,2) M_hat(1,3) M_hat(2,2) M_hat(2,3) M_hat(3,3) ...
        (M_hat*transmitter_pos_hat.').' ...
         transmitter_pos_hat*M_hat*transmitter_pos_hat.'];


% set NLP
t_f = 30; % [s]  ( first estimated mission time )
d_t = 5; % [m]
% ObjectiveWeights = [1 0.1 10]; % weigths used for last experiments
ObjectiveWeights = [1 1e-06 1];
% ObjectiveWeights = [1 1e-10/N 1e-08/N];
% ObjectiveWeights = [100 1e-10/N 1e-08/N];
% ObjectiveWeights = [1 1e-02 1e-01];
OF = buildObjectiveFunction(ObjectiveWeights,N,TIME_STEP,N_approx_bernstain);
problem.objective = OF;
problem.solver = 'fmincon';

% problem.options = optimoptions("fmincon", ...
%                 "Display","iter-detailed");

problem.options = optimoptions("fmincon",...
                "Display","iter-detailed",...
                "Algorithm","interior-point");

if ~USE_NMPC || NLP_AND_NMPC
    if NLP_PLANNING
        last_pos_ode = recievers_pos_ode(:,[1 2 4 5]);
        if CODE_GENERATED_NLP
            NLPOnline_mex; % solve NLP from c generated code;
        else
            NLPOnline; % solve NLP
        end
        
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

% ObjectiveWeights = [1 1e-6/N 1e-02];
ObjectiveWeights = [1 1e-10/N 1e-08/N];

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
% my_temp = H_function(N,recievers_pos);
my_temp = H_function(N,[recievers_pos(:,1:2) zeros(N,1)]);
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
    uncertain_time_range = 40*TIME_STEP; % [s]
    estimation_device_pos = [0 -35 0];
    AS = Dispatcher(N,TIME_STEP,recievers_pos_ode,NaN,NaN);
end

% initialize UAV-network system
network_matrix = [1 1 0 0 0;
                  1 1 1 0 0;
                  0 1 1 1 0;
                  0 0 1 1 1;
                  0 0 0 1 1];
% network_matrix = ones(5,5);
% network_matrix = [1 1 0 0;
%                   1 1 1 0;
%                   0 1 1 1;
%                   0 0 1 1];
% network_matrix = [1 1;
%                   1 1];
% check matrix dimensions
if size(network_matrix,1) ~= N || size(network_matrix,2) ~= N
    fprintf("ERROR: network matrix has wrong dimensions\n");
    return;
end

if exist("UAV_trajs","var")
UAV_NET = UAVNetwork(N, ...
                     TIME_STEP, ...
                     network_matrix, ...
                     recievers_pos_ode, ...
                     X_hat, ...
                     S_RLS, ...
@(Ns,x) Y_function(Ns,x,transmitter_real_pos.',[a b],[M_real(1,1:3) M_real(2,2:3) M_real(3,3)],0,0), ...
                     Drone_NMPC, ...
                     d_safe, ...
                     v_max, ...
                     "3D", ...
                     model_params, ...
                     UAV_trajs);
else
UAV_NET = UAVNetwork(N, ...
                     TIME_STEP, ...
                     network_matrix, ...
                     recievers_pos_ode, ...
                     X_hat, ...
                     S_RLS, ...
@(Ns,x) Y_function(Ns,x,transmitter_real_pos.',[a b],[M_real(1,1:3) M_real(2,2:3) M_real(3,3)],0,0), ...
                     Drone_NMPC, ...
                     d_safe, ...
                     v_max, ...
                     "3D", ...
                     model_params);
end

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
% sim = client.getObject('sim');
sim = client.require('sim');

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

% hide targets
for i = 1:N
    sim.setObjectInt32Param(UAV_handels{i,2},sim.objintparam_visibility_layer,0);
end

% %show targets
% for i = 1:N
%     sim.setObjectInt32Param(UAV_handels{i,2},sim.objintparam_visibility_layer,1);
% end

% show planner device position
PlannerDevice = sim.getObject("./Planner");
CPSIM_node_graph = sim.getObject("./NodeGraph");
if exist("estimation_device_pos","var")
    
    % set PlannerDevice position
    sim.setObjectInt32Param(PlannerDevice,sim.objintparam_visibility_layer,1);
    temp_cell = cell(1,3);
    for j = 1:3; temp_cell{j} = estimation_device_pos(j); end
    temp_cell{3} = 0.5;
    sim.setObjectPosition(PlannerDevice,temp_cell);
    
    % set nodegraph position
    sim.setObjectInt32Param(CPSIM_node_graph,sim.objintparam_visibility_layer,1);
    temp_cell = cell(1,3);
    for j = 1:3; temp_cell{j} = estimation_device_pos(j); end
    temp_cell{3} = 1;
    sim.setObjectPosition(CPSIM_node_graph,temp_cell);
else
    sim.setObjectInt32Param(PlannerDevice,sim.objintparam_visibility_layer,0);
    sim.setObjectInt32Param(CPSIM_node_graph,sim.objintparam_visibility_layer,0);
end


% show centralized estimated position
CPSIM_estimated_position = sim.getObject("./EstimatedPos");
CPSIM_estimated_position_z = sim.getObject("./EstimatedPosz");
for j = 1:3; temp_cell{j} = transmitter_pos_hat(j); end
temp_cell{3} = 0.05;
sim.setObjectPosition(CPSIM_estimated_position,temp_cell);
temp_cell{3} = transmitter_pos_hat(3);
sim.setObjectPosition(CPSIM_estimated_position_z,temp_cell);


% initial inputs (gravity compensation to hover)
for i=1:N
    control_UAV(sim,UAV_propellers_list{i},[1.2753,1.2753,1.2753,1.2753]);
end

% client.setStepping(true);
sim.setStepping(true);

sim.startSimulation();


%% MAINLOOP EXECUTION

BROADCAST_COUNT_DOWN = -1;
DISPATCH_TRAJECTORY_COUNTER = -1*ones(N,2);
DISPATCH_STOP_COUNTER = -1*ones(N,1);
PLANNING_PHASE_COUNTER = -1;

% start video recording
if RECORD_VIDEO
    this_time = string(datetime('now'));
    vid_name = "MV-"+this_time;
    vid_name = regexprep(vid_name," ",":");
    vid_name = regexprep(vid_name,":","-");
    writerObj = VideoWriter(vid_name,'Motion JPEG AVI');
    % set the seconds per image
    approx_magnitude = 1;
    writerObj.FrameRate = 1/TIME_STEP;
    writerObj.Quality = 85;
    % open the writer
    open(writerObj);
end

STEP = 1;
K_STEP = 2;
plotComputedTrajs;
visualize_init;
while t_simulation(STEP) < t_simulation(end)

    if ~USE_NMPC || NLP_AND_NMPC
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

    
    % event-triggered replanning (sync case)
    if ~COMPUTING_DEVICE_DELAY

        if  (NLP_PLANNING || DOUBLE_PHASE || NLP_AND_NMPC) && t_replanning <= 0
            ESTIMATE_ERROR = norm(transmitter_pos_hat-last_replanning_transmitter_pos_hat);
            OPI = OI_function(recievers_pos_ode_history(1:STEP,:,1:2));
            if  ESTIMATE_ERROR > RHO_THRESHOLD || ...
                OPI <= SIGMA_TRESHOLD
                last_replanning_transmitter_pos_hat = transmitter_pos_hat;
                K_STEP = 2; % reset steps
    
                last_pos_ode = recievers_pos_ode(:,[1 2 4 5]);
    
                if CODE_GENERATED_NLP
                    NLPOnline_mex; % solve NLP from c generated code;
                else
                    NLPOnline; % solve NLP
                end
                
                % broadcast new trajectories
                if NLP_AND_NMPC
                    for i = 1:UAV_NET.NUM_UAVS
                        UAV_NET.UAVS{i}.trajectory_ref = squeeze(UAV_trajs(i,:,:));
                        UAV_NET.UAVS{i}.trajectory_step = 1;
                    end
                end
    
                % show computed trajs
                plotComputedTrajs;
    %             % plot new estimated mission endtime
                set(VIZ_END_MISSION_TIME,"String","Estimated endtime: "+string(t_simulation(STEP)+t_f)+" s");
                drawnow;
            end
            t_replanning = 10; % reset timer
        end

    end

    % event-triggered replanning (async case)
    if COMPUTING_DEVICE_DELAY

        if PLANNING_PHASE_COUNTER == -1
            if  (NLP_PLANNING || DOUBLE_PHASE || NLP_AND_NMPC) && t_replanning <= 0
    
                ESTIMATE_ERROR = norm(transmitter_pos_hat-last_replanning_transmitter_pos_hat);
                OPI = OI_function(recievers_pos_ode_history(1:STEP,:,1:2));
                
                if  ESTIMATE_ERROR > RHO_THRESHOLD || OPI <= SIGMA_TRESHOLD
                    
                    % communicate UAV to stop
                    for i = 1:N
                        random_alpha = rand(1,1);
                        delta_spawn_time = ( norm(recievers_pos(i,1:3) - estimation_device_pos(1:3))/medium_velocity ) + random_alpha * uncertain_time_range;
                        DISPATCH_STOP_COUNTER(i) = fix(delta_spawn_time/TIME_STEP)+2;
                    end
                    
                    % to be sure that the trajectories will be near
                    % obstacles
                    PLANNING_PHASE_COUNTER = fix((3*uncertain_time_range)/TIME_STEP)+1;
        
                end
                t_replanning = 10; % reset timer
            end
        elseif PLANNING_PHASE_COUNTER > 0
            PLANNING_PHASE_COUNTER = PLANNING_PHASE_COUNTER - 1;
        elseif PLANNING_PHASE_COUNTER == 0
            last_replanning_transmitter_pos_hat = transmitter_pos_hat;
            K_STEP = 2; % reset steps
    
            last_pos_ode = [available_pos_ode(:,1:2) zeros(size(last_pos_ode,1),2)];
    
            % compute new trajectoryes
            if CODE_GENERATED_NLP
                NLPOnline_mex; % solve NLP from c generated code;
            else
                NLPOnline; % solve NLP
            end
    
            % prepare broadcast
            BROADCAST_COUNT_DOWN = 0; % set to 0 then we are neglecting computational time
    
            % show computed trajs
            plotComputedTrajs;
    %             % plot new estimated mission endtime
            set(VIZ_END_MISSION_TIME,"String","Estimated endtime: "+string(t_simulation(STEP)+t_f)+" s");
            drawnow;
            t_replanning = 10; % reset replanning
            PLANNING_PHASE_COUNTER = -1;
        end

        % handle stop signal arrival to UAVS
        if any(DISPATCH_STOP_COUNTER(:) >= 0)
            for j=1:N
                if DISPATCH_STOP_COUNTER(j) == 0
                    UAV_NET.stop_ith_UAV(j);
                    DISPATCH_STOP_COUNTER(j) = DISPATCH_STOP_COUNTER(j) - 1;
                elseif DISPATCH_STOP_COUNTER(j,1) > 0
                    DISPATCH_STOP_COUNTER(j) = DISPATCH_STOP_COUNTER(j) - 1;
                end
            end
        end
        
        % actual broadcast after cpu-time
        if BROADCAST_COUNT_DOWN > 0; BROADCAST_COUNT_DOWN = BROADCAST_COUNT_DOWN - 1;
        elseif BROADCAST_COUNT_DOWN == 0
            for i = 1:N
                random_alpha = rand(1,1);
                delta_spawn_time = ( norm(recievers_pos(i,1:3) - estimation_device_pos(1:3))/medium_velocity ) + random_alpha * uncertain_time_range;
                DISPATCH_TRAJECTORY_COUNTER(i,1) = fix(delta_spawn_time/TIME_STEP)+2;
                DISPATCH_TRAJECTORY_COUNTER(i,2) = delta_spawn_time;
            end
            BROADCAST_COUNT_DOWN = -1;
        end
    
        % handle trajectory arrival to UAVS
        if any(DISPATCH_TRAJECTORY_COUNTER(:,1) >= 0)
            for j=1:N
                if DISPATCH_TRAJECTORY_COUNTER(j,1) == 0
                    UAV_NET.UAVS{j}.trajectory_ref = squeeze(UAV_trajs(j,:,:));
    %                 UAV_NET.UAVS{j}.trajectory_step = fix((CPU_TIME+DISPATCH_TRAJECTORY_COUNTER(j,2))/TIME_STEP)+1;
                    UAV_NET.UAVS{j}.trajectory_step = 1;
                    DISPATCH_TRAJECTORY_COUNTER(j,1) = DISPATCH_TRAJECTORY_COUNTER(j,1) - 1;
                    DISPATCH_TRAJECTORY_COUNTER(j,2) = 0; 
                elseif DISPATCH_TRAJECTORY_COUNTER(j,1) > 0
                    DISPATCH_TRAJECTORY_COUNTER(j,1) = DISPATCH_TRAJECTORY_COUNTER(j,1) - 1;
                end
            end
        end

    end
    
    % async case preliminarities
    if COMPUTING_DEVICE_DELAY
        % compute centralized async system step
        available_pos_ode = extractPosOde(AS.pull_latest_info((STEP-1)*TIME_STEP));
        available_pos = available_pos_ode(:,1:3);

        % compute new values ( with async sys measurment ) centralized
        H_num = H_function(N,available_pos);
        Y_num = Y_function(N,available_pos,transmitter_real_pos.',[a b],[M_real(1,1:3) M_real(2,2:3) M_real(3,3)],0,0);

        % compute new values decentralized
        UAV_NET.DRLS(beta_ff,Y_num,H_num,true,(STEP-1)*TIME_STEP);
    else 
        % compute new values centralized
        H_num = H_function(N,recievers_pos);
        Y_num = Y_function(N,recievers_pos,transmitter_real_pos.',[a b],[M_real(1,1:3) M_real(2,2:3) M_real(3,3)],0,0);

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

    
    UAV_NET.refresh_nmpc_state(recievers_pos_ode,false);
    if ~USE_NMPC
        % feedback linearization step
        U_FL = UAV_NET.FL_UAV_TEAM_step();
        % hierarcical control step
%         U_FL = UAV_NET.HC_UAV_TEAM_step();
        for i = 1:N
            control_UAV(sim,UAV_propellers_list{i},U_FL(i,:));
        end
    else
        % NMPC step
        U_NMPC = UAV_NET.NMPC_UAV_TEAM_step();
        for i = 1:N
            control_UAV(sim,UAV_propellers_list{i},U_NMPC(i,:));
        end
    end

    % coppelia sim integration step
%     client.step();
    sim.step();
    for i = 1:N
        % get new recievers pos_ode
        recievers_pos_ode(i,:) = getState(sim,UAV_handels{i,1});
        % targets in position estimate
        temp_cell = cell(1,3);
        for j = 1:3; temp_cell{j} = UAV_NET.UAVS{i}.X_ref(j); end
        sim.setObjectPosition(UAV_handels{i,2},sim.handle_world,temp_cell);
        % show centralized estimated position
        for j = 1:3; temp_cell{j} = transmitter_pos_hat(j); end
        temp_cell{3} = 0.05;
        sim.setObjectPosition(CPSIM_estimated_position,temp_cell);
        temp_cell{3} = transmitter_pos_hat(3);
        sim.setObjectPosition(CPSIM_estimated_position_z,temp_cell);
    end
    recievers_pos = recievers_pos_ode(:,1:3);
    UAV_NET.refresh_nmpc_state(recievers_pos_ode,false);

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
%     my_temp = H_function(N,recievers_pos);
    my_temp = H_function(N,[recievers_pos(:,1:2) zeros(N,1)]);
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

    % visualize step
    visualize_update;

end

sim.stopSimulation();

hold off;

% stop and save video recorded
if RECORD_VIDEO
    if ~exist('./Movies', 'dir')
           mkdir('./Movies');
    end
    close(writerObj);
    movefile(vid_name+".avi",'./Movies/.');
end

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
lgnd = cell(1,N);
for i = 1:N
    lgnd{i} = "UAV "+num2str(i);
end
legend(lgnd,"Location","Best");
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
plot(t_simulation(1:STEP),OI_VAL,"Color","Black","LineWidth",2.0);
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
plot(t_simulation(1:STEP),TRANSMITTER_ESTIMATE_VARIATION,"Color","Black","LineWidth",2.0);
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
plot(t_simulation(1:STEP),TRANSMITTER_ESTIMATE_ERROR,"Color","Black","LineWidth",2.0);
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
    plot(t_simulation(1:STEP),TRANSMITTER_ESTIMATE_DRLS_DEVIATION(:,i).',"Color",color_list(i),"LineWidth",2.0);
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
        plot(t_simulation(1:STEP-1),UAV_NET.UAVS{i}.CPU_TIME(1:STEP-1),"Color",color_list(i),"LineWidth",2.0);
        
    end
    plot(t_simulation(1:STEP-1),repmat(Drone_NMPC.Ts,STEP-1,1),"--","Color","black","LineWidth",1.5);
    lgnd = cell(1,N+1);
    for i = 1 :N
        lgnd{i} = "UAV "+num2str(i);
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
    
    abg_cpsim = sim.getObjectOrientation(UAV,sim.handle_world);
    [yaw,pitch,roll] = sim.alphaBetaGammaToYawPitchRoll(abg_cpsim{1},abg_cpsim{2},abg_cpsim{3});
    rpy = [roll pitch yaw];

%     [dx,w] = sim.getObjectVelocity(UAV+sim.handleflag_axis,sim.handle_world);    
%     [dx_no_flag,w_no_flag] = sim.getObjectVelocity(UAV,sim.handle_world);
    [dx,w] = sim.getObjectVelocity(UAV + sim.handleflag_axis);
    dx_mat = zeros(1,3);
    w_mat = zeros(1,3);
    for i = 1:3
        dx_mat(i) = double(dx{i});
        w_mat(i) = double(w{i});
    end
    w_mat = (R.' * w_mat.').';
    UAV_state = double([x dx_mat w_mat rpy]);
end

function control_UAV(sim,UAV_propeller_list,controls)
    for i = 1:length(UAV_propeller_list)
        sim.setJointTargetVelocity(UAV_propeller_list(i),controls(i));
    end
end