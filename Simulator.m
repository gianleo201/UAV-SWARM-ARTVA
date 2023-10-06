clear;
EnvINIT; % initialize environment
MagneticFieldSensors; % build Symbolically H matrix

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
    temp_K{i} = [5 0 20  0;
                 0 5  0 20]; % control gains
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

a = 1;
b = 2;
M_real = diag([b^2 a^2 a^2]);

% RLS algorithm setup
beta_ff = 0.95; % forgetly factor
% RLS algorithm initialization
S_RLS = diag(repmat(10,1,10)); % initial covariance matrix
% transmitter_pos_hat = [0 0 0]; % initial guess for the transmitter
last_replanning_transmitter_pos_hat = transmitter_pos_hat; % keep memory at each replanning
M_hat = [1   0      0;
         0     2    0;
         0     0    2];
% m_11 m_12 m_13 m_22 m_23 m_33 p_t_bar rho
X_hat = [M_hat(1,1) M_hat(1,2) M_hat(1,3) M_hat(2,2) M_hat(2,3) M_hat(3,3) ...
        (M_hat*transmitter_pos_hat.').' ...
         transmitter_pos_hat*M_hat*transmitter_pos_hat.'];


% set NLP
t_f = 30; % [s]  ( first estimated mission time )
d_t = 5; % [m]
if ~exist('rs_dist','var')
    d_safe = 3;
else
    d_safe = rs_dist*0.75;
    fprintf("Security distance among UAVs set to: %d\n",d_safe);
end
v_max = 5; % [m/s]
% ObjectiveWeights = [1 0.1 10]; % weigths used for last experiments
ObjectiveWeights = [1 0.1 10];
OF = buildObjectiveFunction(ObjectiveWeights,N,TIME_STEP,N_approx_bernstain);
problem.objective = OF;
problem.solver = 'fmincon';
problem.options = optimoptions('fmincon', ...
                'Display','iter-detailed', ...
                'OptimalityTolerance',0.5);

if NLP_PLANNING
    NLPOnline; % solve NLP
else
%     L_t = 0.75*hl_length; % for radial exploration
    L_t = hl_length; % for v-h exploration
    et = 2*( L_t /(0.5*v_max)); % cruise velocity at 50%
    trap_num_samples = fix( et / TIME_STEP ) + 1;
    UAV_trajs = zeros(N,trap_num_samples,4);
    for i=1:N
%         reciever_END = reciever_INIT(i,1:2) + L_t * [cos((i-0.5)*angle_sector) sin((i-0.5)*angle_sector)]; % for radial exploration
        reciever_END = reciever_INIT(i,1:2) + hl_length * [0 1]; % for vertical exploration
%         reciever_END = reciever_INIT(i,1:2) + hl_length * [1 0]; % for horizontal exploration
        
        % extract linear path direction
        UAV_path_dir = reciever_END-reciever_INIT(i,1:2);
        UAV_path_dir = UAV_path_dir/norm(UAV_path_dir);
        
        % compute straight path trapezoidal trajectories
        [q, qd, ~, tSamples, ~] = trapveltraj( ...
            [ 0 L_t ], ...
            trap_num_samples, ...
            "PeakVelocity",0.5*v_max, ...
            "EndTime",et);
        UAV_trajs(i,:,1:2) = repmat(reciever_INIT(i,1:2),trap_num_samples,1)+[UAV_path_dir(1)*q.' UAV_path_dir(2)*q.'];
        UAV_trajs(i,:,3:4) = [UAV_path_dir(1)*qd.' UAV_path_dir(2)*qd.'];
    end
end

% % plot ex
% plot(15);
% grid on; hold on;
% szz = size(UAV_trajs);
% vls = zeros(1,szz(2));
% for i=1:length(vls)
%     vls(i) = norm(squeeze(UAV_trajs(1,i,3:4)));
% end
% plot(tSamples,vls);
% hold off;

% event-triggered replanning thresholds
t_replanning = 10; % replanning timer [s]
RHO_THRESHOLD = 2;
SIGMA_TRESHOLD = 0.7;


% compute number of pairs of interdistances
NUM_DIST = 0;
for i = 1:N
    for j = i+1:N
        NUM_DIST = NUM_DIST + 1;
    end
end
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
my_temp = H_function(recievers_pos);
my_temp = my_temp*my_temp.';
last_matrix_sum = my_temp;
my_temp = sqrt(svds(last_matrix_sum,1,"smallestnz"));
OI_VAL = [my_temp];

% estimate variation
TRANSMITTER_ESTIMATE_VARIATION = [0];
ESTIMATE_VARIATION_THRESHOLD = 1e-04;

% estimate error
TRANSMITTER_ESTIMATE_ERROR = [norm(transmitter_real_pos(1:2)-transmitter_pos_hat(1:2))];

%% MAINLOOP EXECUTION

STEP = 1;
K_STEP = 2;

% start video recording
if RECORD_VIDEO
    this_time = string(datetime('now'));
    vid_name = "MV-"+this_time;
    vid_name = regexprep(vid_name," ",":");
    vid_name = regexprep(vid_name,":","-");
    writerObj = VideoWriter(vid_name,'MPEG-4');
    % set the seconds per image
    approx_magnitude = 1;
    writerObj.FrameRate = 1/TIME_STEP;
    % open the writer
    open(writerObj);
end

% show computed trajs
plotComputedTrajs;
visualize_init;
while t_simulation(STEP) < t_simulation(end)

    % ending condition on estimate variation
    if STEP > 1 && TRANSMITTER_ESTIMATE_VARIATION(end) < ESTIMATE_VARIATION_THRESHOLD
        break;
    end

    % augment simulation-data list
    INTER_DISTANCES = [INTER_DISTANCES;zeros(1,NUM_DIST)];
    VELOCITIES = [VELOCITIES;zeros(1,N)];


%     % if replanning timer is over run another optimization problem
%     if t_replanning <= 0
%         K_STEP = 2; % reset steps
%         NLPOnline; % solve again OCP
%         t_replanning = t_f-TIME_STEP; % reset timer
%         % show computed trajs
%         plotComputedTrajs;
%         % plot new estimated mission endtime
%         set(VIZ_END_MISSION_TIME,"String","Estimated endtime: "+string(t_simulation(STEP)+t_f)+" s");
%     end

    % event-Triggered replanning
    if  (NLP_PLANNING || DOUBLE_PHASE) && t_replanning <= 0
        ESTIMATE_ERROR = norm(transmitter_pos_hat-last_replanning_transmitter_pos_hat);
        OPI = OI_function(recievers_pos_ode_history(1:STEP,:,1:2));
        if  ESTIMATE_ERROR > RHO_THRESHOLD || ...
            OPI <= SIGMA_TRESHOLD

            K_STEP = 2; % reset steps
            NLPOnline; % solve again NLP
            % show computed trajs
            plotComputedTrajs;
            % plot new estimated mission endtime
            set(VIZ_END_MISSION_TIME,"String","Estimated endtime: "+string(t_simulation(STEP)+t_f)+" s");
        end
            t_replanning = t_f-TIME_STEP; % reset timer
    end

    
    % compute new values
    H_num = H_function(recievers_pos);
    Y_num = Y_function(recievers_pos,transmitter_real_pos.',[a b],[M_real(1,1:3) M_real(2,2:3) M_real(3,3)],0,0);
    
    % RLS algorithm step
    X_hat = ( X_hat.' +  inv(S_RLS)*H_num*(Y_num-H_num.'*X_hat.') ).'; % new estimate of parameters vector;
    S_RLS = beta_ff * S_RLS + H_num*H_num.';

    % extract transmitter position estimate
    M_hat = [X_hat(1) X_hat(2) X_hat(3);X_hat(2) X_hat(4) X_hat(5); X_hat(3) X_hat(5) X_hat(6)];
    old_transmitter_pos_hat = transmitter_pos_hat;
    transmitter_pos_hat = (inv(M_hat)*X_hat(7:9).').'; % new transmitter estimate

    % set current reference point for UAVS
%     reshape(squeeze(UAV_trajs(:,K_STEP,:)).',1,[]).'
%     repmat([transmitter_pos_hat(1:2) 0 0 0],1,N).'
    temp_dim = size(UAV_trajs);
    if K_STEP > temp_dim(2) % if trajs have ended stay at the end
        curr_traj = squeeze(UAV_trajs(:,end,:));
        curr_traj(:,3:4) = zeros(N,2);
        UAV_references = reshape(curr_traj.',1,[]).';
    else
        UAV_references = reshape(squeeze(UAV_trajs(:,K_STEP,:)).',1,[]).';
    end
    % simulate the UAVS
    step_results = ode45( ...
    UAV_TEAM(UAV_references,K_uavs_gain,total_A_matrix,total_B_matrix), ...
          [0 TIME_STEP], ...
          reshape(recievers_pos_ode.', 1, []));
    recievers_pos_ode = reshape(step_results.y(:,end),4,N).';
    recievers_pos = [ recievers_pos_ode(:,1:2) zeros(N,1)];

    % compute interdistances
    k = 1;
    for i = 1:N
        VELOCITIES(end,i) = norm(recievers_pos_ode(i,3:4));
        for j = i+1:N
            INTER_DISTANCES(end,k) = norm(recievers_pos(i,:)-recievers_pos(j,:));
            k = k + 1;
        end
    end

    % prepare next step
    STEP = STEP + 1;
    K_STEP = K_STEP + 1;
    t_replanning = t_replanning-TIME_STEP;
    
    % update states history
    recievers_pos_ode_history(STEP,:,:) = recievers_pos_ode;

    % update Observability index value
    my_temp = H_function(recievers_pos);
    my_temp = my_temp*my_temp.';
    last_matrix_sum = last_matrix_sum + (my_temp - last_matrix_sum)/STEP;
    new_OI = sqrt(svds(last_matrix_sum,1,"smallestnz"));
    OI_VAL = [OI_VAL new_OI];

    % update estimate variation
    my2_temp = norm(transmitter_pos_hat(1:2)-old_transmitter_pos_hat(1:2));
    TRANSMITTER_ESTIMATE_VARIATION = [TRANSMITTER_ESTIMATE_VARIATION my2_temp];
    TRANSMITTER_ESTIMATE_ERROR = [TRANSMITTER_ESTIMATE_ERROR norm(transmitter_real_pos(1:2)-transmitter_pos_hat(1:2))];

    % visualize step
    pause(TIME_STEP);
    visualize_update;

end

hold off;

% stop and save video recorded
if RECORD_VIDEO
    if ~exist('./Movies', 'dir')
           mkdir('./Movies')
    end
    close(writerObj);
    movefile(vid_name+".mp4",'./Movies/.');
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
legend("1-2","1-3","2-3");
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
save TRANSMITTER_ESTIMATE_ERROR.mat TRANSMITTER_ESTIMATE_ERROR;
saveas(fig6,'./tmp_dir/TEE.png');
saveas(fig6,'./tmp_dir/TEE.fig');
saveas(fig6,'./tmp_dir/TEE','epsc');


%% UAV DYNAMICS ODE

function UAV_TEAM_ODE = UAV_TEAM(x_ref,K,p1,p2)
    UAV_TEAM_ODE = @(t,x) p1*x+p2*K*(x_ref-x);
end
