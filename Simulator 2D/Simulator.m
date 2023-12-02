
clear;
EnvINIT; % initialize environment

%% UAVS DYNAMICS (symilified to double integrators)

% imageine point robot. Can act with them with 2 cartesian normal forces Fx
% Fy

generic_A_matrix = [0 0 1 0;
                    0 0 0 1;
                    0 0 0 0;
                    0 0 0 0];
generic_B_matrix = [0 0;
                    0 0;
                    1 0;
                    0 1];
temp_A = cell(N,1);
temp_B = cell(N,1);
temp_K = cell(N,1);

% set PD controller
for i=1:N
    temp_A{i} = generic_A_matrix;
    temp_B{i} = generic_B_matrix;
    temp_K{i} = [15 0 20  0;
                 0 15  0 20]; % control gains
end

total_A_matrix = blkdiag(temp_A{:});
total_B_matrix = blkdiag(temp_B{:});
K_uavs_gain = blkdiag(temp_K{:});

%% MAINLOOP SETUP

% robots initial position setup
recievers_pos = reciever_INIT;
recievers_pos_ode = [reciever_INIT(:,1:2) zeros(N,2)];
recievers_pos_ode_history = zeros(END_TIME/TIME_STEP,N,4);
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
S_RLS = diag([1 1 1 1 1 1 ...
             (hl_length^2)/12 (hl_length^2)/12 10 ...
             (hl_length^4)/9]); % initial covariance matrix
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
% ObjectiveWeights = [1 1e-05/N 1e-04/N];
ObjectiveWeights = [1 1e-10/N 1e-04/N];
OF = buildObjectiveFunction(ObjectiveWeights,N,TIME_STEP,N_approx_bernstain);
problem.objective = OF;
problem.solver = 'fmincon';

% problem.options = optimoptions("fmincon",...
%                 "Algorithm","active-set",...
%                 "Display","iter-detailed");

problem.options = optimoptions("fmincon",...
                "Display","iter-detailed");

% problem.options = optimoptions("fmincon",...
%                 "Algorithm","active-set",...
%                 "Display","iter-detailed",...
%                 "EnableFeasibilityMode",true,...
%                 "SubproblemAlgorithm","cg");

if ~USE_NMPC
    if NLP_PLANNING
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

% change weights for faster computation
ObjectiveWeights = [1 1e-07/N 1e-08/N];

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
    VELOCITIES(end,i) = norm(recievers_pos_ode(i,3:4));
    for j = i+1:N
        INTER_DISTANCES(end,k) = norm(recievers_pos(i,:)-recievers_pos(j,:));
        k = k + 1;
    end
end

% obsevability index
my_temp = H_function(N,recievers_pos);
my_temp = my_temp*my_temp.';
last_matrix_sum = my_temp;
my_temp = min_sv_O(last_matrix_sum,min([N,10]));
% OI_VAL = [my_temp];
OI_VAL = [0];

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
% network_matrix = [1 1 0;
%                   1 1 1;
%                   0 1 1];
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
@(Ns,x) Y_function(Ns,x,transmitter_real_pos.',[a b],[M_real(1,1:3) M_real(2,2:3) M_real(3,3)],0,0), ...
                     Drone_NMPC, ...
                     d_safe, ...
                     v_max);

% DRLS estimate variation from standard RLS
NETWORK_ESTIMATES = UAV_NET.pull_estimates();
NETWORK_ESTIMATES_DEV = zeros(1,N);
for i = 1 : N
    NETWORK_ESTIMATES_DEV(i) = norm(NETWORK_ESTIMATES(i,1:2)-transmitter_pos_hat(1:2));
end
TRANSMITTER_ESTIMATE_DRLS_DEVIATION = [NETWORK_ESTIMATES_DEV];

%% MAINLOOP EXECUTION

STEP = 1;
K_STEP = 2;

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

% show computed trajs
plotComputedTrajs;
visualize_init;
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
            if CODE_GENERATED_NLP
                NLPOnline_mex; % solve NLP from c generated code;
            else
                NLPOnline; % solve NLP
            end
            % show computed trajs
            plotComputedTrajs;
            % plot new estimated mission endtime
            set(VIZ_END_MISSION_TIME,"String","Estimated endtime: "+string(t_simulation(STEP)+t_f)+" s");
        end
        t_replanning = 10; % reset timer
    
    end
    
    % async case preliminarities
    if COMPUTING_DEVICE_DELAY
        % compute centralized async system step
        available_pos_ode = extractPosOde(AS.pull_latest_info((STEP-1)*TIME_STEP));
        available_pos = [available_pos_ode(:,1:2) zeros(size(available_pos_ode,1),1)];

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


    if ~USE_NMPC
        % set current reference point for UAVS
%         reshape(squeeze(UAV_trajs(:,K_STEP,:)).',1,[]).'
%         repmat([transmitter_pos_hat(1:2) 0 0 0],1,N).'
        temp_dim = size(UAV_trajs);
        if K_STEP > temp_dim(2) % if trajs have ended stay at the end
            curr_traj = squeeze(UAV_trajs(:,end,:));
            curr_traj(:,3:4) = zeros(N,2);
            UAV_references = reshape(curr_traj.',1,[]).';
        else
            UAV_references = reshape(squeeze(UAV_trajs(:,K_STEP,:)).',1,[]).';
        end

        % simulate the UAVS (standard PD controller)
        step_results = ode45( ...
        UAV_TEAM(UAV_references,K_uavs_gain,total_A_matrix,total_B_matrix), ...
              [0 TIME_STEP], ...
              reshape(recievers_pos_ode.', 1, []));
    else
        U_NMPC = UAV_NET.NMPC_UAV_TEAM_step();
        step_results = ode45( ...
        UAV_TEAM_NMPC(U_NMPC.',total_A_matrix,total_B_matrix), ...
              [0 TIME_STEP], ...
              reshape(recievers_pos_ode.', 1, []));
    end

    % extract integration results
    recievers_pos_ode = reshape(step_results.y(:,end),4,N).';
    recievers_pos = [ recievers_pos_ode(:,1:2) zeros(N,1)];

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
            delta_spawn_time = ( norm(recievers_pos(i,1:2) - estimation_device_pos(1:2))/medium_velocity ) + random_alpha * uncertain_time_range;
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
        VELOCITIES(end,i) = norm(recievers_pos_ode(i,3:4));
        for j = i+1:N
            INTER_DISTANCES(end,k) = norm(recievers_pos(i,:)-recievers_pos(j,:));
            k = k + 1;
        end
    end
  
    % update states history
    recievers_pos_ode_history(STEP,:,:) = recievers_pos_ode;

    % update Observability index value
    my_temp = H_function(N,recievers_pos);
    my_temp = my_temp*my_temp.';
    last_matrix_sum = last_matrix_sum + (my_temp - last_matrix_sum)/STEP;
    new_OI = min_sv_O(last_matrix_sum,min([N,10]));
    OI_VAL = [OI_VAL new_OI];

    % update estimate error and variation
    my2_temp = norm(transmitter_pos_hat(1:2)-old_transmitter_pos_hat(1:2));
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

    % visualize step
    pause(TIME_STEP);
    visualize_update;

end

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
plot(t_simulation(1:STEP),repmat(SIGMA_TRESHOLD,STEP,1),"--","Color","red","LineWidth",1.5);
hold off;
legend("O.I","O.I. threshold","Location","Best");
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
    plot(t_simulation(1:STEP-1),repmat(TIME_STEP,STEP-1,1),"--","Color","black","LineWidth",1.5);
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

%% UAV DYNAMICS ODE

function UAV_TEAM_ODE = UAV_TEAM(x_ref,K,p1,p2)
    UAV_TEAM_ODE = @(t,x) p1*x+p2*K*(x_ref-x);
end

function UAV_TEAM_ODE = UAV_TEAM_NMPC(U_NMPC,p1,p2)
    UAV_TEAM_ODE = @(t,x) p1*x+p2*U_NMPC;
end
